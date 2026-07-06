using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR.ARSubsystems;

namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    /// <summary>
    /// Monitors geospatial anchor drift and requests anchor recreation when it is
    /// both statistically significant (above the GPS noise floor) and persistent
    /// (debounced across consecutive checks), or when a localization jump occurs.
    /// Recreation is deferred while current GPS accuracy is worse than it was when
    /// the anchors were registered — rebuilding with noisier GPS re-pins a larger error.
    /// </summary>
    public class DriftCorrector : MonoBehaviour
    {
        [Header("Dependencies")]
        [Tooltip("Earth Manager for tracking state and GPS positioning")]
        public AREarthManager earthManager;

        [Header("Drift Correction Settings")]
        [Tooltip("Enable automatic drift correction")]
        public bool enableDriftCorrection = true;

        [Tooltip("Time between drift checks (seconds)")]
        [Range(5f, 30f)]
        public float driftCorrectionInterval = 10f;

        [Tooltip("Request recreation when persistent median drift exceeds this (meters)")]
        [Range(3f, 15f)]
        public float recreateAnchorThreshold = 5f;

        [Tooltip("Median drift above this in a single check is treated as a localization jump (needs 2 consecutive confirmations instead of the full debounce)")]
        [Range(15f, 50f)]
        public float localizationJumpThreshold = 20f;

        [Tooltip("Anchor lifetime before refresh is considered (0 = infinite). Refresh still waits for good accuracy.")]
        [Range(0f, 300f)]
        public float anchorLifetimeSeconds = 0f;

        [Tooltip("Skip drift checks when HorizontalAccuracy is worse than this (meters)")]
        [Range(1f, 20f)]
        public float requiredAccuracy = 10f;

        [Tooltip("Skip drift checks when heading uncertainty exceeds this (degrees). Yaw error dominates lateral drift in GPS-only mode.")]
        [Range(5f, 30f)]
        public float maxYawAccuracyDegrees = 15f;

        [Tooltip("Measured drift must exceed this multiple of current HorizontalAccuracy to count as real")]
        [Range(0.5f, 3f)]
        public float accuracyNoiseMultiplier = 1.5f;

        [Tooltip("Consecutive over-threshold checks required before recreation is requested")]
        [Range(1, 5)]
        public int requiredConsecutiveDetections = 3;

        [Tooltip("If recreation is requested but anchors are not cleared/re-registered within this time, the corrector unblocks itself")]
        public float recreationTimeoutSeconds = 15f;

        [Header("Events")]
        public UnityEvent OnExtremeDriftDetected;
        [System.Serializable]
        public class DriftUpdatedEvent : UnityEvent<float> { }
        public DriftUpdatedEvent OnDriftUpdated;

        /// <summary>Exponentially smoothed median drift (meters), or -1 before the first measurement.</summary>
        public float CurrentDriftEstimate { get; private set; } = -1f;

        private class MonitoredAnchor
        {
            public ARGeospatialAnchor anchor;
            public GPSCoordinate originalCoordinate;
            public float creationTime;
            public float accuracyAtRegistration;
        }

        private readonly List<MonitoredAnchor> _monitoredAnchors = new List<MonitoredAnchor>();
        private readonly List<double> _driftSamples = new List<double>();

        private float _lastDriftCheckTime;
        private bool _isRecreatingAnchors;
        private float _recreationRequestTime;
        private int _consecutiveDriftDetections;
        private int _consecutiveJumpDetections;

        // EMA weight for CurrentDriftEstimate
        private const float SmoothingFactor = 0.3f;
        // Meters of slack when comparing current accuracy against accuracy at registration
        private const float AccuracyRefinementMargin = 2f;

        public void RegisterAnchor(ARGeospatialAnchor anchor, GPSCoordinate originalCoordinate)
        {
            if (anchor == null) return;

            // Only delay the first check when monitoring starts fresh; per-anchor
            // registration on long routes must not defer checks indefinitely.
            if (_monitoredAnchors.Count == 0)
                _lastDriftCheckTime = Time.time;

            float accuracy = float.MaxValue;
            if (earthManager != null && earthManager.EarthTrackingState == TrackingState.Tracking)
                accuracy = (float)earthManager.CameraGeospatialPose.HorizontalAccuracy;

            _monitoredAnchors.Add(new MonitoredAnchor
            {
                anchor = anchor,
                originalCoordinate = originalCoordinate,
                creationTime = Time.time,
                accuracyAtRegistration = accuracy
            });
        }

        public void ClearAnchors()
        {
            _monitoredAnchors.Clear();
            _isRecreatingAnchors = false;
            _consecutiveDriftDetections = 0;
            _consecutiveJumpDetections = 0;
        }

        public void NotifyRecreatingAnchors()
        {
            _isRecreatingAnchors = true;
            _recreationRequestTime = Time.time;
        }

        void Update()
        {
            if (!enableDriftCorrection || _monitoredAnchors.Count == 0) return;

            // Failsafe: if recreation was requested but nothing cleared/re-registered
            // anchors (e.g. OnExtremeDriftDetected is not wired), unblock monitoring.
            if (_isRecreatingAnchors)
            {
                if (Time.time - _recreationRequestTime > recreationTimeoutSeconds)
                {
                    Debug.LogError("[DriftCorrector] Recreation requested but never completed. " +
                        "Wire OnExtremeDriftDetected to NavigationController.RecreateAllAnchors. Resuming monitoring.");
                    _isRecreatingAnchors = false;
                }
                return;
            }

            if (earthManager == null || earthManager.EarthTrackingState != TrackingState.Tracking) return;

            if (Time.time - _lastDriftCheckTime > driftCorrectionInterval)
            {
                _lastDriftCheckTime = Time.time;
                PerformDriftCorrection();
            }

            if (anchorLifetimeSeconds > 0)
                CheckAnchorLifetime();
        }

        public void ForceDriftCorrection()
        {
            if (!enableDriftCorrection || _monitoredAnchors.Count == 0 || _isRecreatingAnchors) return;
            PerformDriftCorrection();
        }

        private void PerformDriftCorrection()
        {
            if (earthManager == null || earthManager.EarthTrackingState != TrackingState.Tracking) return;

            _monitoredAnchors.RemoveAll(m => m.anchor == null);
            if (_monitoredAnchors.Count == 0) return;

            GeospatialPose currentPose = earthManager.CameraGeospatialPose;
            float horizontalAccuracy = (float)currentPose.HorizontalAccuracy;

            if (horizontalAccuracy > requiredAccuracy)
            {
                Debug.Log($"[DriftCorrector] Skipping check — GPS accuracy {horizontalAccuracy:F2}m > {requiredAccuracy:F2}m");
                return;
            }

            if (currentPose.OrientationYawAccuracy > maxYawAccuracyDegrees)
            {
                Debug.Log($"[DriftCorrector] Skipping check — yaw accuracy {currentPose.OrientationYawAccuracy:F1}° > {maxYawAccuracyDegrees:F1}°");
                return;
            }

            _driftSamples.Clear();
            foreach (var monitored in _monitoredAnchors)
            {
                Pose anchorWorldPose = new Pose(
                    monitored.anchor.transform.position,
                    monitored.anchor.transform.rotation);
                GeospatialPose anchorPose = earthManager.Convert(anchorWorldPose);

                _driftSamples.Add(GeoUtils.CalculateDistance(
                    new GPSCoordinate(anchorPose.Latitude, anchorPose.Longitude, anchorPose.Altitude),
                    monitored.originalCoordinate));
            }

            if (_driftSamples.Count == 0) return;

            // Median is robust to a single badly-tracked anchor skewing the estimate.
            _driftSamples.Sort();
            float medianDrift = (float)_driftSamples[_driftSamples.Count / 2];

            CurrentDriftEstimate = CurrentDriftEstimate < 0f
                ? medianDrift
                : Mathf.Lerp(CurrentDriftEstimate, medianDrift, SmoothingFactor);
            OnDriftUpdated?.Invoke(medianDrift);

            // Drift below the GPS noise floor is indistinguishable from measurement noise.
            float effectiveThreshold = Mathf.Max(recreateAnchorThreshold, accuracyNoiseMultiplier * horizontalAccuracy);

            if (medianDrift >= localizationJumpThreshold)
            {
                _consecutiveJumpDetections++;
                _consecutiveDriftDetections = 0;
                Debug.LogWarning($"[DriftCorrector] Possible localization jump: {medianDrift:F2}m ({_consecutiveJumpDetections}/2 confirmations)");
                if (_consecutiveJumpDetections >= 2)
                    TryTriggerRecreation(horizontalAccuracy, $"localization jump ({medianDrift:F2}m)");
            }
            else if (medianDrift > effectiveThreshold)
            {
                _consecutiveDriftDetections++;
                _consecutiveJumpDetections = 0;
                Debug.LogWarning($"[DriftCorrector] Drift {medianDrift:F2}m > {effectiveThreshold:F2}m " +
                    $"({_consecutiveDriftDetections}/{requiredConsecutiveDetections} consecutive, GPS accuracy {horizontalAccuracy:F2}m)");
                if (_consecutiveDriftDetections >= requiredConsecutiveDetections)
                    TryTriggerRecreation(horizontalAccuracy, $"persistent drift ({medianDrift:F2}m)");
            }
            else
            {
                _consecutiveDriftDetections = 0;
                _consecutiveJumpDetections = 0;
            }
        }

        private void CheckAnchorLifetime()
        {
            foreach (var monitored in _monitoredAnchors)
            {
                if (monitored.anchor == null) continue;
                if (Time.time - monitored.creationTime > anchorLifetimeSeconds)
                {
                    float accuracy = (float)earthManager.CameraGeospatialPose.HorizontalAccuracy;
                    TryTriggerRecreation(accuracy, "anchor lifetime expired");
                    return;
                }
            }
        }

        private void TryTriggerRecreation(float currentAccuracy, string reason)
        {
            // Rebuilding anchors with noisier GPS than they were created with
            // re-pins a larger error; wait until accuracy is at least as good.
            float bestRegistrationAccuracy = float.MaxValue;
            foreach (var m in _monitoredAnchors)
                if (m.accuracyAtRegistration < bestRegistrationAccuracy)
                    bestRegistrationAccuracy = m.accuracyAtRegistration;

            if (currentAccuracy > bestRegistrationAccuracy + AccuracyRefinementMargin)
            {
                Debug.LogWarning($"[DriftCorrector] Recreation deferred ({reason}): current accuracy " +
                    $"{currentAccuracy:F2}m is worse than at registration ({bestRegistrationAccuracy:F2}m).");
                _consecutiveDriftDetections = 0;
                _consecutiveJumpDetections = 0;
                return;
            }

            Debug.LogWarning($"[DriftCorrector] Requesting anchor recreation: {reason}");
            _isRecreatingAnchors = true;
            _recreationRequestTime = Time.time;
            _consecutiveDriftDetections = 0;
            _consecutiveJumpDetections = 0;
            OnExtremeDriftDetected?.Invoke();
        }
    }
}
