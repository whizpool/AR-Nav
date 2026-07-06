namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Detects when the user is moving in the wrong direction (away from
    /// the current target waypoint). Triggers after sustained wrong-way
    /// movement to avoid false alarms from brief GPS jumps.
    /// </summary>
    public class WrongWayDetector
    {
        #region Private Fields (Configuration & State)
        // --- Configuration ---
        private readonly float _angleTolerance;        // degrees from "directly away"
        private readonly float _sustainedDurationSec;  // seconds of wrong-way before alert
        private readonly float _minSpeedMps;           // ignore if user is barely moving

        // --- State ---
        private float _wrongWayAccumulator = 0f;
        private bool  _isWrongWay = false;
        #endregion

        #region Public Properties
        /// <summary>True when the user has been walking the wrong way long enough.</summary>
        public bool IsWrongWay => _isWrongWay;
        #endregion

        #region Constructor
        /// <param name="angleTolerance">
        ///   How many degrees from "directly away" still counts as wrong-way.
        ///   e.g., 60 means any heading within ±60° of "directly away" is wrong.
        /// </param>
        /// <param name="sustainedDurationSec">
        ///   How many seconds of sustained wrong-way walking before triggering.
        ///   Prevents GPS jitter from causing false alarms.
        /// </param>
        /// <param name="minSpeedMps">
        ///   Minimum speed (m/s) to consider the user "moving".
        ///   Below this, we assume stationary and don't flag wrong-way.
        /// </param>
        public WrongWayDetector(
            float angleTolerance = 60f,
            float sustainedDurationSec = 4f,
            float minSpeedMps = 0.5f)
        {
            _angleTolerance = angleTolerance;
            _sustainedDurationSec = sustainedDurationSec;
            _minSpeedMps = minSpeedMps;
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Call every frame/GPS update.
        /// </summary>
        /// <param name="userMovementDir">Smoothed 2D movement direction (East, North).</param>
        /// <param name="userSpeed">User speed in m/s.</param>
        /// <param name="directionToWaypoint">2D vector FROM user TO target waypoint.</param>
        /// <param name="deltaTime">Time.deltaTime or GPS update interval.</param>
        /// <returns>True if wrong-way alert should be shown.</returns>
        public bool Update(
            Vector2 userMovementDir, float userSpeed,
            Vector2 directionToWaypoint, float deltaTime)
        {
            // Not moving fast enough to judge direction
            if (userSpeed < _minSpeedMps || userMovementDir.sqrMagnitude < 0.001f)
            {
                // Slowly decay the accumulator while stationary
                _wrongWayAccumulator = Mathf.Max(0f, _wrongWayAccumulator - deltaTime * 0.5f);
                return _isWrongWay;
            }

            // Angle between movement direction and direction-to-waypoint
            // 0° = walking directly toward waypoint
            // 180° = walking directly away
            float angle = Vector2.Angle(userMovementDir, directionToWaypoint);

            if (angle > (180f - _angleTolerance))
            {
                // User is walking roughly away from the waypoint
                _wrongWayAccumulator += deltaTime;
            }
            else
            {
                // User is heading toward or lateral — decay accumulator
                _wrongWayAccumulator = Mathf.Max(0f, _wrongWayAccumulator - deltaTime * 2f);
            }

            _isWrongWay = _wrongWayAccumulator >= _sustainedDurationSec;
            return _isWrongWay;
        }

        /// <summary>Call when advancing to a new waypoint.</summary>
        public void Reset()
        {
            _wrongWayAccumulator = 0f;
            _isWrongWay = false;
        }
        #endregion
    }
}
