namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;
    using UnityEngine.XR.ARSubsystems;

    /// <summary>
    /// Smoothed heading provider that blends Geospatial heading with VIO-derived
    /// heading to reduce jitter from raw compass data.
    /// </summary>
    public class SmoothedHeading : MonoBehaviour
    {
        #region Serialized Fields
        [SerializeField] private AREarthManager _earthManager;
        [Range(0.01f, 1f)]
        [SerializeField] private float smoothingFactor = 0.1f; // lower = smoother
        #endregion

        #region Private Fields
        private float _smoothedHeading;
        private bool  _initialized;
        #endregion

        #region Public Properties
        /// <summary>Current smoothed heading in degrees (0 = North, 90 = East).</summary>
        public float Heading => _smoothedHeading;
        #endregion

        #region Unity Lifecycle
        void Update()
        {
            if (_earthManager == null || _earthManager.EarthTrackingState != TrackingState.Tracking)
                return;

            float rawHeading = _earthManager.CameraGeospatialPose.EunRotation == Quaternion.identity
                ? _smoothedHeading // skip bad frames
                : QuaternionToYaw(_earthManager.CameraGeospatialPose.EunRotation);

            if (!_initialized)
            {
                _smoothedHeading = rawHeading;
                _initialized = true;
                return;
            }

            // Exponential moving average with circular wrap handling
            float delta = Mathf.DeltaAngle(_smoothedHeading, rawHeading);
            _smoothedHeading = Mathf.Repeat(
                _smoothedHeading + delta * smoothingFactor, 360f);
        }
        #endregion

        #region Private Helper Methods
        private float QuaternionToYaw(Quaternion q)
        {
            // EUN rotation: Y axis = Up, extract yaw
            Vector3 euler = q.eulerAngles;
            return euler.y;
        }
        #endregion
    }
}
