using UnityEngine;

/// <summary>
/// Geodetic navigation helper — finds a road intersection point given:
///   • Your current position + heading (Ray 1)
///   • Either the destination point after the turn  (Mode A — angle-free)
///     OR an explicit turn angle in degrees          (Mode B — angle-known)
///
/// Mode A  TryFindIntersection(device, heading, destination, out result)
///   The post-turn bearing is derived from the bearing device→destination
///   projected onto the post-heading plane.  No turn angle needed.
///   Works for ANY turn angle (30°, 60°, 90°, 120° …).
///
/// Mode B  TryFindIntersection(device, heading, destination, turnAngle, out result)
///   You supply the signed turn angle explicitly (negative = left, positive = right).
///   The solver uses that to fix Ray 2's direction, then finds where it hits Ray 1.
///   Use this when you have the junction angle from map data and the destination
///   point is approximate.
///
/// Accuracy: sub-metre for distances < 10 km (planar ENU approximation).
/// </summary>
public static class GeoNavIntersection
{
    // ── Constants ────────────────────────────────────────────────────────────
    private const double EarthRadius = 6_371_000.0; // metres (mean radius)

    // ═════════════════════════════════════════════════════════════════════════
    // MODE A — angle-free (destination point drives post-turn bearing)
    // ═════════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Find the intersection without knowing the turn angle in advance.
    ///
    /// How it works:
    ///   Ray 1 starts at the device and goes along <paramref name="headingDeg"/>.
    ///   Ray 2 starts at <paramref name="destLat"/>/<paramref name="destLng"/>
    ///   and goes in the direction that is the REVERSE of the bearing from
    ///   device to destination — i.e. it shoots "back up the post-turn road"
    ///   toward the intersection.
    ///   The intersection of the two rays is the turn point.
    ///
    /// This handles any turn angle automatically because the geometry is
    /// determined entirely by the two known points and the heading.
    /// </summary>
    /// <param name="deviceLat">Device latitude  (degrees)</param>
    /// <param name="deviceLng">Device longitude (degrees)</param>
    /// <param name="headingDeg">Travel bearing 0–360, 0 = North clockwise</param>
    /// <param name="destLat">Destination latitude  — point AFTER the turn</param>
    /// <param name="destLng">Destination longitude — point AFTER the turn</param>
    /// <param name="result">Populated on success</param>
    /// <returns>True when a valid forward intersection exists</returns>
    public static bool TryFindIntersection(
        double deviceLat, double deviceLng,
        double headingDeg,
        double destLat,   double destLng,
        out IntersectionResult result)
    {
        result = default;

        // Bearing from destination back toward the intersection.
        // The post-turn road runs from intersection → destination, so the
        // reverse (destination → intersection) is exactly the back-bearing
        // of device→destination, rotated 180°.
        double deviceToDestBrng = InitialBearing(deviceLat, deviceLng, destLat, destLng);
        double backBrng         = NormalizeBearing(deviceToDestBrng + 180.0);

        return SolveRayIntersection(
            deviceLat, deviceLng, headingDeg,
            destLat, destLng, backBrng,
            ref result);
    }

    /// <summary>Vector2 convenience overload for Mode A (x = lat, y = lng).</summary>
    public static bool TryFindIntersection(
        Vector2 deviceCoord,
        float   headingDeg,
        Vector2 destCoord,
        out IntersectionResult result)
        => TryFindIntersection(
            deviceCoord.x, deviceCoord.y,
            headingDeg,
            destCoord.x, destCoord.y,
            out result);

    // ═════════════════════════════════════════════════════════════════════════
    // MODE B — explicit turn angle
    // ═════════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Find the intersection when you know the signed turn angle at the junction.
    ///
    /// The post-turn bearing is:  heading + turnAngleDeg
    ///   e.g.  heading 45°, turn −60° (left)  →  post-turn 345°
    ///         heading 45°, turn +30° (right)  →  post-turn  75°
    ///
    /// Ray 2 starts at <paramref name="destLat"/>/<paramref name="destLng"/>
    /// and runs in the reverse of that post-turn bearing back toward the intersection.
    /// </summary>
    /// <param name="turnAngleDeg">
    /// Signed turn angle in degrees.
    /// Negative = left  (e.g. −90 for a standard left turn).
    /// Positive = right (e.g. +45 for a gentle right).
    /// </param>
    public static bool TryFindIntersection(
        double deviceLat, double deviceLng,
        double headingDeg,
        double destLat,   double destLng,
        double turnAngleDeg,
        out IntersectionResult result)
    {
        result = default;

        double postTurnBrng = NormalizeBearing(headingDeg + turnAngleDeg);
        double backBrng     = NormalizeBearing(postTurnBrng + 180.0);

        return SolveRayIntersection(
            deviceLat, deviceLng, headingDeg,
            destLat, destLng, backBrng,
            ref result);
    }

    /// <summary>Vector2 convenience overload for Mode B.</summary>
    public static bool TryFindIntersection(
        Vector2 deviceCoord,
        float   headingDeg,
        Vector2 destCoord,
        float   turnAngleDeg,
        out IntersectionResult result)
        => TryFindIntersection(
            deviceCoord.x, deviceCoord.y,
            headingDeg,
            destCoord.x, destCoord.y,
            turnAngleDeg,
            out result);

    // ═════════════════════════════════════════════════════════════════════════
    // Core ray–ray solver (shared by both modes)
    // ═════════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Solves two rays in the local ENU plane:
    ///   Ray 1 — origin: device,  direction: headingDeg
    ///   Ray 2 — origin: dest,    direction: backBrng  (shoots back toward intersection)
    ///
    /// Returns true and populates <paramref name="result"/> when a valid
    /// forward intersection (t ≥ 0) is found.
    /// </summary>
    private static bool SolveRayIntersection(
        double deviceLat, double deviceLng,
        double headingDeg,
        double destLat,   double destLng,
        double backBrng,
        ref IntersectionResult result)
    {
        // Destination in local ENU metres (device is the origin)
        Vector2d destLocalENU = GeoToLocalENU(deviceLat, deviceLng, destLat, destLng);
        Vector2d headingDir = BearingToDirection(headingDeg); // Ray 1 direction
        Vector2d backBrngDir = BearingToDirection(backBrng);  // Ray 2 direction

        // Solve: A + distDeviceToIntersection·headingDir = destLocalENU + distDestToIntersection·backBrngDir   (A = origin = 0,0)
        //   →   distDeviceToIntersection·headingDir - distDestToIntersection·backBrngDir = destLocalENU
        //
        // Matrix form: | headingDir.x  -backBrngDir.x | · | distDeviceToIntersection |   | destLocalENU.x |
        //              | headingDir.y  -backBrngDir.y |   | distDestToIntersection   | = | destLocalENU.y |
        //
        // Cramer's rule:
        double denominator = headingDir.x * (-backBrngDir.y) - headingDir.y * (-backBrngDir.x);

        if (System.Math.Abs(denominator) < 1e-10)
        {
            Debug.LogWarning("[GeoNavIntersection] Rays are parallel — " +
                             "heading and post-turn direction are collinear. " +
                             "Check that the turn angle is not 0° or 180°.");
            return false;
        }

        double distDeviceToIntersection = (destLocalENU.x * (-backBrngDir.y) - destLocalENU.y * (-backBrngDir.x)) / denominator; // dist device→intersection
        double distDestToIntersection = (headingDir.x * destLocalENU.y    - headingDir.y * destLocalENU.x)    / denominator; // dist dest→intersection

        if (distDeviceToIntersection < 0.0)
        {
            Debug.LogWarning("[GeoNavIntersection] Intersection is behind the device " +
                             $"(distDeviceToIntersection = {distDeviceToIntersection:F1} m). Heading may be pointing away from the junction.");
            return false;
        }

        // Intersection in local ENU, then back to geo
        ENUToGeo(deviceLat, deviceLng,
                 distDeviceToIntersection * headingDir.x, distDeviceToIntersection * headingDir.y,
                 out double intersectionLat, out double intersectionLng);

        double actualPostTurnBrng = InitialBearing(intersectionLat, intersectionLng, destLat, destLng);

        // Signed turn angle: how much you actually turn at the intersection.
        // Normalise to (−180, +180] so negative = left, positive = right.
        double rawTurn   = actualPostTurnBrng - headingDeg;
        double turnAngle = ((rawTurn + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

        result = new IntersectionResult
        {
            Latitude                   = intersectionLat,
            Longitude                  = intersectionLng,
            DistanceToIntersection     = distDeviceToIntersection,
            DistanceIntersectionToDest = System.Math.Abs(distDestToIntersection),
            PostTurnBearing            = actualPostTurnBrng,
            TurnAngle                  = turnAngle,
        };

        return true;
    }

    // ── Public geodetic utilities ─────────────────────────────────────────────

    /// <summary>Haversine distance in metres.</summary>
    public static double HaversineDistance(
        double lat1, double lng1,
        double lat2, double lng2)
    {
        double deltaLatRad = Deg2Rad(lat2 - lat1);
        double deltaLngRad = Deg2Rad(lng2 - lng1);
        double chordLengthSquared = System.Math.Sin(deltaLatRad / 2) * System.Math.Sin(deltaLatRad / 2)
                                  + System.Math.Cos(Deg2Rad(lat1)) * System.Math.Cos(Deg2Rad(lat2))
                                  * System.Math.Sin(deltaLngRad / 2) * System.Math.Sin(deltaLngRad / 2);
        return 2.0 * EarthRadius * System.Math.Asin(System.Math.Sqrt(chordLengthSquared));
    }

    /// <summary>Initial bearing from p1 to p2, degrees [0, 360).</summary>
    public static double InitialBearing(
        double lat1, double lng1,
        double lat2, double lng2)
    {
        double startLatRad = Deg2Rad(lat1), endLatRad = Deg2Rad(lat2);
        double deltaLngRad = Deg2Rad(lng2 - lng1);
        double yCoord = System.Math.Sin(deltaLngRad) * System.Math.Cos(endLatRad);
        double xCoord = System.Math.Cos(startLatRad) * System.Math.Sin(endLatRad)
                      - System.Math.Sin(startLatRad) * System.Math.Cos(endLatRad) * System.Math.Cos(deltaLngRad);
        return NormalizeBearing(Rad2Deg(System.Math.Atan2(yCoord, xCoord)));
    }

    /// <summary>Destination point given start, bearing (degrees) and distance (metres).</summary>
    public static void DestinationPoint(
        double lat, double lng, double bearingDeg, double distanceMetres,
        out double destLat, out double destLng)
    {
        double angularDistance = distanceMetres / EarthRadius;
        double startLatRad = Deg2Rad(lat), startLngRad = Deg2Rad(lng), bearingRad = Deg2Rad(bearingDeg);

        double destLatRad = System.Math.Asin(
            System.Math.Sin(startLatRad) * System.Math.Cos(angularDistance) +
            System.Math.Cos(startLatRad) * System.Math.Sin(angularDistance) * System.Math.Cos(bearingRad));

        double destLngRad = startLngRad + System.Math.Atan2(
            System.Math.Sin(bearingRad) * System.Math.Sin(angularDistance) * System.Math.Cos(startLatRad),
            System.Math.Cos(angularDistance) - System.Math.Sin(startLatRad) * System.Math.Sin(destLatRad));

        destLat = Rad2Deg(destLatRad);
        destLng = (Rad2Deg(destLngRad) + 540.0) % 360.0 - 180.0;
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    private static Vector2d GeoToLocalENU(
        double originLat, double originLng,
        double targetLat, double targetLng)
    {
        double dLat = Deg2Rad(targetLat - originLat);
        double dLng = Deg2Rad(targetLng - originLng);
        return new Vector2d(
            dLng * EarthRadius * System.Math.Cos(Deg2Rad(originLat)), // East
            dLat * EarthRadius);                                        // North
    }

    private static void ENUToGeo(
        double originLat, double originLng,
        double eastMetres, double northMetres,
        out double lat, out double lng)
    {
        lat = originLat + Rad2Deg(northMetres / EarthRadius);
        lng = originLng + Rad2Deg(eastMetres  / (EarthRadius * System.Math.Cos(Deg2Rad(originLat))));
    }

    private static Vector2d BearingToDirection(double bearingDeg)
    {
        double rad = Deg2Rad(bearingDeg);
        return new Vector2d(System.Math.Sin(rad), System.Math.Cos(rad));
    }

    private static double NormalizeBearing(double deg)
        => ((deg % 360.0) + 360.0) % 360.0;

    private static double Deg2Rad(double degrees) => degrees * System.Math.PI / 180.0;
    private static double Rad2Deg(double radians) => radians * 180.0 / System.Math.PI;

    private readonly struct Vector2d
    {
        public readonly double x, y;
        public Vector2d(double x, double y) { this.x = x; this.y = y; }
    }
}

// ── Result struct ─────────────────────────────────────────────────────────────

/// <summary>Output from a successful intersection solve.</summary>
[System.Serializable]
public struct IntersectionResult
{
    /// <summary>Latitude of the intersection (degrees).</summary>
    public double Latitude;

    /// <summary>Longitude of the intersection (degrees).</summary>
    public double Longitude;

    /// <summary>Distance in metres from the device to the intersection.</summary>
    public double DistanceToIntersection;

    /// <summary>Distance in metres from the intersection to the destination.</summary>
    public double DistanceIntersectionToDest;

    /// <summary>
    /// Bearing to follow after the turn, degrees [0, 360).
    /// Computed as the actual bearing from intersection → destination.
    /// </summary>
    public double PostTurnBearing;

    /// <summary>
    /// Signed turn angle at the intersection, degrees in (−180, +180].
    ///   Negative = left  turn  (e.g. −90, −45, −120)
    ///   Positive = right turn  (e.g. +30, +90)
    /// </summary>
    public double TurnAngle;

    /// <summary>True when TurnAngle &lt; 0 (left turn).</summary>
    public bool IsLeftTurn  => TurnAngle < 0.0;

    /// <summary>True when TurnAngle &gt; 0 (right turn).</summary>
    public bool IsRightTurn => TurnAngle > 0.0;

    public override string ToString() =>
        $"Intersection ({Latitude:F6}, {Longitude:F6})  " +
        $"| {(IsLeftTurn ? "LEFT" : "RIGHT")} {System.Math.Abs(TurnAngle):F1}°  " +
        $"| DeviceDist: {DistanceToIntersection:F1} m  " +
        $"| DestDist: {DistanceIntersectionToDest:F1} m  " +
        $"| PostTurnBearing: {PostTurnBearing:F1}°";
}