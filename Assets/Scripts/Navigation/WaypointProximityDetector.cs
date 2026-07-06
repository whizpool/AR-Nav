namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Detects when the user has "reached" a waypoint, accounting for GPS drift.
    /// Uses a two-stage approach:
    ///   1. Activation radius: start paying attention
    ///   2. Confirmation: closest approach or pass-by detection
    /// </summary>
    public class WaypointProximityDetector
    {
        public event System.Action<string> OnStatusUpdated;

        private void LogStatus(string status)
        {
            OnStatusUpdated?.Invoke(status);
        }

        #region Private Fields (Configuration)
        // --- Configuration ---
        private float baseActivationRadius = 20f;
        private float baseConfirmRadius    = 10f;
        private float minRadius     = 4f;   // floor even with perfect GPS
        private float maxRadius     = 25f;  // ceiling for terrible GPS
        private float accuracyScale = 1.5f; // multiplier on horizontalAccuracy
        #endregion

        #region Private Fields (State)
        // --- State ---
        private enum Phase { Approaching, Inside }
        private Phase _phase = Phase.Approaching;
        private float _closestDistance = float.MaxValue;
        private int   _framesSinceClosest = 0;
        #endregion

        #region Public Methods
        /// <summary>
        /// Call every GPS update. Returns true when the waypoint should be
        /// considered "reached."
        /// </summary>
        /// <param name="distanceToWaypoint">Haversine distance in meters.</param>
        /// <param name="horizontalAccuracy">
        ///   From AREarthManager.CameraGeospatialPose.HorizontalAccuracy (meters, 68% CI).
        /// </param>
        public bool UpdateProximity(float distanceToWaypoint, float horizontalAccuracy)
        {
            // Scale radii by current GPS quality
            float activation = ComputeRadius(baseActivationRadius, horizontalAccuracy);
            float confirm    = ComputeRadius(baseConfirmRadius, horizontalAccuracy);

            switch (_phase)
            {
                case Phase.Approaching:
                    if (distanceToWaypoint < activation)
                    {
                        _phase = Phase.Inside;
                        _closestDistance = distanceToWaypoint;
                        _framesSinceClosest = 0;
                        LogStatus($"Phase changed: Approaching -> Inside. Distance={distanceToWaypoint:F2}m, activation={activation:F2}m");
                    }
                    break;

                case Phase.Inside:
                    // Track closest approach
                    if (distanceToWaypoint < _closestDistance)
                    {
                        _closestDistance = distanceToWaypoint;
                        _framesSinceClosest = 0;
                        LogStatus($"Closest approach updated: {_closestDistance:F2}m");
                    }
                    else
                    {
                        _framesSinceClosest++;
                    }

                    // --- Trigger conditions (any one is sufficient) ---

                    // 1. Close enough — high confidence
                    if (distanceToWaypoint < confirm)
                    {
                        LogStatus($"Reached: distance {distanceToWaypoint:F2}m < confirm radius {confirm:F2}m");
                        return Reached();
                    }

                    // 2. "Passed by" — user was inside activation, got close,
                    //    and is now moving away for several consecutive updates
                    if (_framesSinceClosest > 3 && _closestDistance < activation * 0.7f)
                    {
                        LogStatus($"Reached: passed by (closest: {_closestDistance:F2}m, frames: {_framesSinceClosest})");
                        return Reached();
                    }

                    // 3. Walked well past — distance is now > activation again.
                    // Also require a genuinely close approach first, so briefly
                    // clipping the outer ring (e.g. walking parallel past a turn,
                    // or a single GPS jump) can't skip the waypoint outright.
                    if (distanceToWaypoint > activation * 1.3f && _closestDistance < activation * 0.7f)
                    {
                        LogStatus($"Reached: walked past activation (distance {distanceToWaypoint:F2}m > {activation * 1.3f:F2}m)");
                        return Reached();
                    }

                    break;
            }

            return false;
        }

        /// <summary>
        /// Reset state for the next waypoint.
        /// </summary>
        public void Reset()
        {
            _phase = Phase.Approaching;
            _closestDistance = float.MaxValue;
            _framesSinceClosest = 0;
            LogStatus("Proximity detector reset to Approaching phase.");
        }
        #endregion

        #region Private Helper Methods
        private bool Reached()
        {
            Reset();
            return true;
        }

        private float ComputeRadius(float baseRadius, float accuracy)
        {
            float scaled = Mathf.Max(baseRadius, accuracy * accuracyScale);
            return Mathf.Clamp(scaled, minRadius, maxRadius);
        }
        #endregion
    }
}
