namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using System.Collections.Generic;
    using UnityEngine;
    using Google.XR.ARCoreExtensions;

    /// <summary>
    /// Utility class for geographical coordinate calculations.
    /// RULE: All geo calculation related utils should ONLY be placed and used from this class.
    /// </summary>
    public static class GeoUtils
    {
        /// <summary>
        /// Interpolates waypoints along a route using full double precision.
        /// FIX: Replaced float Lerp (7 sig figs) with manual double interpolation (15 sig figs).
        /// Float precision alone causes 2–5m coordinate errors at GPS scale.
        /// </summary>
        public static List<GPSCoordinate> InterpolateWaypoints(List<GPSCoordinate> points, float intervalMeters)
        {
            List<GPSCoordinate> result = new List<GPSCoordinate>();

            for (int i = 0; i < points.Count - 1; i++)
            {
                GPSCoordinate start = points[i];
                GPSCoordinate end = points[i + 1];

                double distance = CalculateDistance(start, end);
                int segments = Mathf.Max(1, Mathf.CeilToInt((float)distance / intervalMeters));

                for (int j = 0; j < segments; j++)
                {
                    double t = j / (double)segments;

                    // FIX: Use double arithmetic, NOT Mathf.Lerp (which casts to float)
                    result.Add(new GPSCoordinate(
                        start.latitude + t * (end.latitude - start.latitude),
                        start.longitude + t * (end.longitude - start.longitude),
                        start.altitude + t * (end.altitude - start.altitude)
                    ));
                }
            }

            result.Add(points[points.Count - 1]);
            return result;
        }

        /// <summary>
        /// Calculates bearing from one GPS coordinate to another.
        /// FIX: All trig uses System.Math (double precision), not Mathf (float precision).
        /// </summary>
        public static Quaternion CalculateBearingRotation(GPSCoordinate from, GPSCoordinate to)
        {
            // FIX: Use System.Math for double-precision trig throughout
            double lat1 = from.latitude * System.Math.PI / 180.0;
            double lat2 = to.latitude * System.Math.PI / 180.0;
            double dLon = (to.longitude - from.longitude) * System.Math.PI / 180.0;

            double y = System.Math.Sin(dLon) * System.Math.Cos(lat2);
            double x = System.Math.Cos(lat1) * System.Math.Sin(lat2) -
                       System.Math.Sin(lat1) * System.Math.Cos(lat2) * System.Math.Cos(dLon);

            double bearing = System.Math.Atan2(y, x) * 180.0 / System.Math.PI;

            // ARCore ENU: 0° = East, 90° = North
            return Quaternion.Euler(0, (float)bearing, 0);
        }

        /// <summary>
        /// Haversine great-circle distance.
        /// FIX: All trig uses System.Math (double precision), not Mathf (float precision).
        /// </summary>
        public static double CalculateDistance(GPSCoordinate from, GPSCoordinate to)
        {
            const double R = 6371000.0;

            // FIX: Use System.Math for double-precision trig throughout
            double dLat = (to.latitude - from.latitude) * System.Math.PI / 180.0;
            double dLon = (to.longitude - from.longitude) * System.Math.PI / 180.0;

            double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                       System.Math.Cos(from.latitude * System.Math.PI / 180.0) *
                       System.Math.Cos(to.latitude * System.Math.PI / 180.0) *
                       System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);

            double c = 2.0 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1.0 - a));
            return R * c;
        }

        public static GPSCoordinate OffsetCoordinate(GeospatialPose origin, double northMeters, double eastMeters)
        {
            double latOffset = northMeters / 111320.0;
            double lonOffset = eastMeters / (111320.0 * System.Math.Cos(origin.Latitude * System.Math.PI / 180.0));
            return new GPSCoordinate(origin.Latitude + latOffset, origin.Longitude + lonOffset, origin.Altitude);
        }
    }
}
