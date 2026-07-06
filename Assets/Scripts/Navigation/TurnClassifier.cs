namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    public enum TurnDirection { Straight, Left, Right, UTurn }

    /// <summary>
    /// Classifies the turn at waypoint B given the path A → B → C.
    /// All positions are in local Unity space (X=East, Z=North).
    /// </summary>
    public static class TurnClassifier
    {
        #region Constants
        /// <summary>
        /// Threshold angles (degrees) for classifying a turn.
        /// - Angles less than straightThreshold are "Straight"
        /// - Angles greater than uTurnThreshold are "U-Turn"
        /// - Everything in between is "Left" or "Right"
        /// </summary>
        private const float StraightThreshold = 20f;  // degrees
        private const float UTurnThreshold    = 160f;  // degrees
        #endregion

        #region Public Classification Methods
        /// <summary>
        /// Returns the turn direction and the signed angle at B.
        /// Positive angle = Right turn, Negative angle = Left turn.
        /// </summary>
        public static (TurnDirection direction, float angle) Classify(
            Vector3 a, Vector3 b, Vector3 c)
        {
            // 1. Flatten to 2D (XZ plane — East/North)
            Vector2 incoming = new Vector2(b.x - a.x, b.z - a.z);
            Vector2 outgoing = new Vector2(c.x - b.x, c.z - b.z);

            // Guard: degenerate segments
            if (incoming.sqrMagnitude < 0.01f || outgoing.sqrMagnitude < 0.01f)
                return (TurnDirection.Straight, 0f);

            // 2. Signed angle from incoming to outgoing
            //    Positive = clockwise = Right, Negative = counter-clockwise = Left
            float signedAngle = Vector2.SignedAngle(incoming, outgoing);
            // Note: Vector2.SignedAngle returns [-180, 180].
            // Positive = counter-clockwise in Unity's 2D space.
            // Since our 2D is (East, North) and we want map-style rotation:
            //   Counter-clockwise on map = Left turn
            //   Clockwise on map = Right turn
            // Unity's Vector2.SignedAngle: positive = CCW, so:
            //   positive signedAngle → Left turn
            //   negative signedAngle → Right turn

            float absAngle = Mathf.Abs(signedAngle);

            // 3. Classify
            if (absAngle < StraightThreshold)
                return (TurnDirection.Straight, signedAngle);

            if (absAngle > UTurnThreshold)
                return (TurnDirection.UTurn, signedAngle);

            // Positive SignedAngle → CCW → Left; Negative → CW → Right
            TurnDirection dir = signedAngle > 0 ? TurnDirection.Left : TurnDirection.Right;
            return (dir, signedAngle);
        }

        /// <summary>
        /// Classifies the turn using the user's ACTUAL approach direction
        /// instead of the static route geometry. This correctly handles:
        /// - Overshoot & return (user approaches from opposite side)
        /// - Side-street approaches
        /// - Any non-standard approach angle
        /// </summary>
        /// <param name="userMovementDir">
        ///   Smoothed 2D movement vector (East, North) from UserMovementTracker.
        /// </param>
        /// <param name="waypointPos">The current waypoint position (local XZ).</param>
        /// <param name="nextWaypointPos">The next waypoint position (local XZ).</param>
        public static (TurnDirection direction, float angle) ClassifyDynamic(
            Vector2 userMovementDir,
            Vector3 waypointPos,
            Vector3 nextWaypointPos)
        {
            // The "incoming" is the user's actual direction of travel
            Vector2 incoming = userMovementDir;

            // The "outgoing" is still the route: current waypoint → next waypoint
            Vector2 outgoing = new Vector2(
                nextWaypointPos.x - waypointPos.x,
                nextWaypointPos.z - waypointPos.z);

            if (incoming.sqrMagnitude < 0.001f || outgoing.sqrMagnitude < 0.001f)
                return (TurnDirection.Straight, 0f);

            float signedAngle = Vector2.SignedAngle(incoming, outgoing);
            float absAngle = Mathf.Abs(signedAngle);

            if (absAngle < StraightThreshold)
                return (TurnDirection.Straight, signedAngle);

            if (absAngle > UTurnThreshold)
                return (TurnDirection.UTurn, signedAngle);

            TurnDirection dir = signedAngle > 0 ? TurnDirection.Left : TurnDirection.Right;
            return (dir, signedAngle);
        }
        #endregion
    }
}
