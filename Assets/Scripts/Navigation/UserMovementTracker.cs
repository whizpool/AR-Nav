namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;
    using System.Collections.Generic;

    /// <summary>
    /// Tracks the user's actual movement direction by averaging recent GPS
    /// position deltas. Provides a smoothed 2D heading vector (East, North)
    /// that represents which direction the user is actually walking.
    /// </summary>
    public class UserMovementTracker
    {
        public event System.Action<string> OnStatusUpdated;

        private void LogStatus(string status)
        {
            OnStatusUpdated?.Invoke(status);
        }

        #region Private Structures
        private struct Sample
        {
            public Vector2 position; // (East, North) in local space
            public float   time;
        }
        #endregion

        #region Private Fields
        private readonly Queue<Sample> _samples = new Queue<Sample>();
        private readonly float _windowSeconds;
        private readonly float _minMovementMeters;
        #endregion

        #region Public Properties
        /// <summary>
        /// The user's smoothed movement direction (East, North).
        /// Zero vector if the user is stationary.
        /// </summary>
        public Vector2 SmoothedDirection { get; private set; } = Vector2.zero;

        /// <summary>
        /// True if we have enough movement data to trust the direction.
        /// </summary>
        public bool HasValidHeading { get; private set; } = false;

        /// <summary>
        /// Speed in meters/second (smoothed).
        /// </summary>
        public float Speed { get; private set; } = 0f;
        #endregion

        #region Constructor
        /// <param name="windowSeconds">Time window for averaging (e.g., 3 seconds).</param>
        /// <param name="minMovementMeters">Minimum displacement to consider
        /// the user "moving" (filters out GPS jitter while standing still).</param>
        public UserMovementTracker(float windowSeconds = 3f, float minMovementMeters = 2f)
        {
            _windowSeconds = windowSeconds;
            _minMovementMeters = minMovementMeters;
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Call every GPS update with the user's current local-space position.
        /// </summary>
        public void Update(Vector2 localPositionEN, float time)
        {
            // This runs every frame — only log when HasValidHeading actually
            // flips, and only build the message string when it does, to avoid
            // a per-frame allocation + Debug.Log storm during navigation.
            bool wasValid = HasValidHeading;

            _samples.Enqueue(new Sample { position = localPositionEN, time = time });

            // Trim old samples outside the window
            while (_samples.Count > 0 && _samples.Peek().time < time - _windowSeconds)
                _samples.Dequeue();

            if (_samples.Count < 2)
            {
                HasValidHeading = false;
                Speed = 0f;
                if (wasValid) LogStatus($"Tracker: insufficient samples ({_samples.Count}). Heading invalid.");
                return;
            }

            // Compute displacement from oldest to newest sample in window
            var oldest = _samples.Peek();
            var newest = localPositionEN;
            Vector2 displacement = newest - oldest.position;
            float dt = time - oldest.time;

            if (dt < 0.1f)
            {
                HasValidHeading = false;
                if (wasValid) LogStatus("Tracker: time delta too small. Heading invalid.");
                return;
            }

            float distance = displacement.magnitude;
            Speed = distance / dt;

            if (distance < _minMovementMeters)
            {
                // User is essentially stationary — don't update heading
                HasValidHeading = false;
                if (wasValid) LogStatus($"Tracker: user stationary (distance={distance:F2}m < {_minMovementMeters:F2}m). Heading invalid.");
                return;
            }

            SmoothedDirection = displacement.normalized;
            HasValidHeading = true;
            if (!wasValid) LogStatus($"Tracker: moving (distance={distance:F2}m, speed={Speed:F2}m/s). Heading valid.");
        }

        public void Reset()
        {
            _samples.Clear();
            SmoothedDirection = Vector2.zero;
            HasValidHeading = false;
            Speed = 0f;
            LogStatus("Tracker reset.");
        }
        #endregion
    }
}
