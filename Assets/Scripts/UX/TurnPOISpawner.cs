namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;
    using UnityEngine.XR.ARFoundation;

    /// <summary>
    /// Spawns and manages turn-indicator POIs using a hybrid anchor strategy.
    /// Geospatial pose provides geo-truth; a local ARAnchor provides VIO stability.
    /// </summary>
    public class TurnPOISpawner : MonoBehaviour
    {
        public event System.Action<string> OnStatusUpdated;

        private void LogStatus(string status)
        {
            OnStatusUpdated?.Invoke(status);
        }

        #region Serialized Fields
        [SerializeField] private AREarthManager   _earthManager;
        [SerializeField] private WorldSpacePanelUI _worldSpacePanelPrefab;
        [SerializeField] private Camera            _arCamera;
        [SerializeField] private float             _spawnAheadDistance = 60f; // meters
        [SerializeField] private float             _despawnBehindDistance = 25f;
        #endregion

        #region Private Fields
        private WorldSpacePanelUI _activePOI;
        private ARAnchor          _activeAnchor;

        // Falls back to Camera.main only if the AR camera was never assigned/propagated.
        private Camera ActiveCamera => _arCamera != null ? _arCamera : Camera.main;
        #endregion

        /// <summary>Called by NavigationController once it resolves the XR Origin's camera.</summary>
        public void SetCamera(Camera cam) => _arCamera = cam;

        #region Public Spawning & Refinement Methods
        /// <summary>
        /// Call when the user should see the next turn indicator.
        /// </summary>
        public void SpawnTurnPOI(
            double waypointLat, double waypointLon, double waypointAlt,
            TurnDirection direction)
        {
            DespawnCurrent();

            if (_worldSpacePanelPrefab == null || _earthManager == null) return;

            // 1. Resolve the world position from geospatial pose
            var geoPose = _earthManager.CameraGeospatialPose;

            // Convert waypoint to local space relative to camera's geo-position
            Vector3 localOffset = GeoMath.GeoToLocal(
                waypointLat, waypointLon, waypointAlt,
                geoPose.Latitude, geoPose.Longitude, geoPose.Altitude);

            // Target position in Unity world space
            Vector3 worldTarget = ActiveCamera.transform.position + localOffset;

            // Clamp Y to a comfortable viewing height (e.g., 1.5m above ground)
            worldTarget.y = ActiveCamera.transform.position.y + 0.5f;

            // 2. Create a LOCAL ARAnchor at this position for VIO stability
            var anchorGO = new GameObject("TurnAnchor");
            anchorGO.transform.position = worldTarget;
            anchorGO.transform.rotation = Quaternion.identity;
            _activeAnchor = anchorGO.AddComponent<ARAnchor>();

            // 3. Instantiate the WorldSpacePanelUI
            _activePOI = Instantiate(_worldSpacePanelPrefab, _activeAnchor.transform);
            _activePOI.transform.localPosition = Vector3.zero;
            
            // Ensure the game object is active so its Awake/OnEnable run and UI elements are resolved
            _activePOI.gameObject.SetActive(true);
            
            // Set up the panel UI for the turn
            string turnText = direction switch
            {
                TurnDirection.Left  => "Turn Left",
                TurnDirection.Right => "Turn Right",
                TurnDirection.UTurn => "U-Turn",
                _                   => "Proceed Straight"
            };
            
            _activePOI.SetTitle(turnText);
            _activePOI.SetDescription("At the next waypoint");
            
            // Add billboard script if it's not already there
            if (_activePOI.GetComponent<BillboardToCamera>() == null)
            {
                _activePOI.gameObject.AddComponent<BillboardToCamera>();
            }

            LogStatus($"Spawned turn POI: {turnText} at coordinates ({waypointLat:F6}, {waypointLon:F6}, {waypointAlt:F1}m)");
        }

        /// <summary>
        /// Call periodically (e.g., every 2 seconds) to gently correct
        /// the POI's position based on updated geospatial data.
        /// </summary>
        /// <param name="lerpFraction">
        /// Fraction of the remaining offset to close on this call (0-1), not a
        /// per-second rate — this is called once every few seconds, not per-frame.
        /// </param>
        public void RefineAnchorPosition(
            double waypointLat, double waypointLon, double waypointAlt,
            float lerpFraction = 0.3f)
        {
            // Note: we deliberately do NOT move _activeAnchor.transform here.
            // ARAnchor's transform is owned and overwritten by the AR subsystem
            // every frame to track the physical anchor point; writing to it
            // directly is fought by ARCore and has no lasting effect. Instead we
            // nudge the visible POI's LOCAL offset under the (stable) anchor.
            if (_activeAnchor == null || _activePOI == null) return;

            var geoPose = _earthManager.CameraGeospatialPose;
            Vector3 localOffset = GeoMath.GeoToLocal(
                waypointLat, waypointLon, waypointAlt,
                geoPose.Latitude, geoPose.Longitude, geoPose.Altitude);

            Vector3 correctedWorld = ActiveCamera.transform.position + localOffset;
            correctedWorld.y = ActiveCamera.transform.position.y + 0.5f;

            Vector3 desiredLocal = _activeAnchor.transform.InverseTransformPoint(correctedWorld);

            // Smooth correction — never teleport
            _activePOI.transform.localPosition = Vector3.Lerp(
                _activePOI.transform.localPosition,
                desiredLocal,
                lerpFraction);

            LogStatus($"Refined POI position. New distance to camera: {Vector3.Distance(ActiveCamera.transform.position, _activePOI.transform.position):F2}m");
        }

        public void DespawnCurrent()
        {
            if (_activePOI == null && _activeAnchor == null) return;

            if (_activePOI != null)    Destroy(_activePOI.gameObject);
            if (_activeAnchor != null) Destroy(_activeAnchor.gameObject);
            _activePOI = null;
            _activeAnchor = null;

            LogStatus("Despawned current turn POI and anchor.");
        }
        #endregion
    }
}
