namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using System.Collections;
    using System.Collections.Generic;
    using UnityEngine;
    using UnityEngine.Android;
    using Google.XR.ARCoreExtensions;
    using UnityEngine.XR.ARFoundation;
    using Unity.XR.CoreUtils;
    using UnityEngine.XR.ARSubsystems;
    using UnityEngine.Networking;
    using System.Linq;
    using UnityEngine.UI;
    using TMPro;

    /// <summary>
    /// Enhanced Navigation Controller with drift correction and anchor stabilization.
    /// Fixed for GPS-only regions (no VPS): double precision math, terrain anchors, Roads API snap.
    /// Compatible with Unity 6.3 LTS - uses XROrigin instead of deprecated ARSessionOrigin.
    /// </summary>
    public class NavigationController : MonoBehaviour

    {
        public Text DebugText;

        [Header("UI References")]
        [Tooltip("Heading text UI component")]
        public TextMeshProUGUI headingText;

        [Header("AR References")]
        [Tooltip("Drag your XR Origin here from the scene")]
        public XROrigin xrOrigin;

        [Tooltip("AR Session component for tracking state")]
        public ARSession arSession;

        [Tooltip("Earth Manager for geospatial tracking and GPS positioning")]
        public AREarthManager earthManager;

        [Tooltip("Anchor Manager for creating and managing geospatial anchors")]
        public ARAnchorManager anchorManager;

        [Tooltip("ARCore Extensions component for ARCore API access")]
        public ARCoreExtensions arCoreExtensions;

        [Tooltip("3D arrow prefab to show at each waypoint")]
        public GameObject arrowPrefab;

        [Tooltip("3D terrain prefab to show when terrain anchors are used")]
        public GameObject terrainPrefab;

        [Header("Navigation Settings")]
        [Tooltip("Distance between arrow waypoints in meters")]
        [Range(3f, 20f)]
        public float waypointIntervalMeters = 5f;

        [Tooltip("Maximum distance to show arrows (meters)")]
        [Range(20f, 200f)]
        public float maxVisibleDistance = 50f;

        [Tooltip("Minimum horizontal accuracy required to place anchors (meters). Relax to 10-15 in GPS-only regions.")]
        [Range(1f, 200f)]
        public float requiredAccuracy = 10f;


        [Header("Drift Correction")]
        [Tooltip("Optional DriftCorrector component to monitor and handle drift.")]
        public DriftCorrector driftCorrector;

        [Header("Anchor Settings")]
        [Tooltip("Use terrain anchors for altitude stability (recommended for GPS-only regions, no VPS needed)")]
        public bool useTerrainAnchors = true;

        // Rooftop anchors require VPS — disabled in GPS-only regions
        // public bool useRooftopAnchors = false;

        [Header("Debug Settings")]
        public bool enableDebugLogs = true;
        public bool showAccuracyInUI = true;
        public bool showDriftDebugSpheres = false;
        public Material driftDebugMaterial;

        [Tooltip("Auto-create the hard-coded demo route on startup. Turn OFF for production builds.")]
        public bool autoCreateTestRoute = true;

        // Private components
        private Camera arCamera;

        // Path management
        private List<AnchorData> pathAnchors = new List<AnchorData>();
        private List<GPSCoordinate> currentRoute = new List<GPSCoordinate>();

        // Consistent altitude for all anchors in a single route batch.
        // Captured once at the start of BuildAndPlaceAnchors to prevent per-anchor
        // altitude jitter caused by GPS altitude fluctuations during sequential creation.
        private double _routeBatchAltitude = double.MinValue;

        // Drift correction
        private bool isRecreatingAnchors = false;

        // Permission tracking
        private bool locationPermissionGranted = false;
        private bool locationPermissionRequested = false;

        private bool _hasRefinedAnchors = false;

        // Tracking state
        private bool isInitialized = false;

        // UI callback delegates
        public System.Action<string> OnStatusUpdate;
        public System.Action<bool> OnTrackingStateChanged;
        public System.Action<float> OnAccuracyUpdate;

        [Header("Turn Navigation & UI")]
        public TurnPOISpawner poiSpawner;
        public GameObject wrongWayUI;

        // --- Waypoint Navigation State ---
        private int _currentWaypointIndex = 0;
        private WaypointProximityDetector _proximity = new WaypointProximityDetector();
        private UserMovementTracker _movement = new UserMovementTracker(windowSeconds: 3f, minMovementMeters: 2f);
        private WrongWayDetector _wrongWay = new WrongWayDetector(angleTolerance: 60f, sustainedDurationSec: 4f);
        private float _refinementTimer = 0f;
        private const float RefinementInterval = 2f;
        private bool _poiActive = false;
        private bool _waypointNavActive = false;
        private bool _isBuildingPath = false;
        private Coroutine _buildRoutine;

        // Cached to avoid a per-iteration allocation while placing anchors.
        private readonly WaitForSeconds _anchorPlacementDelay = new WaitForSeconds(0.2f);

        private TurnDirection _displayedTurn = TurnDirection.Straight;
        private float _lastReclassifyTime = -999f;
        private const float ReclassifyCooldown = 2f;
        private const float ReclassifyMarginDeg = 10f; // must clear TurnClassifier's 20° straight threshold decisively

        private double _refLat, _refLon, _refAlt;

        #region Data Structures

        [System.Serializable]
        private class AnchorData
        {
            public ARGeospatialAnchor anchor;
            public GPSCoordinate originalCoordinate;
            public GameObject visualObject;
            public float creationTime;
            public GameObject driftDebugSphere;

            public AnchorData(ARGeospatialAnchor anchor, GPSCoordinate coord, GameObject visual)
            {
                this.anchor = anchor;
                this.originalCoordinate = coord;
                this.visualObject = visual;
                this.creationTime = Time.time;
            }
        }

        // ── Roads API response models ──────────────────────────────────────────

        [System.Serializable]
        private class SnapToRoadsResponse
        {
            public List<SnappedPoint> snappedPoints;
        }

        [System.Serializable]
        private class SnappedPoint
        {
            public SnappedLatLng location;
        }

        [System.Serializable]
        private class SnappedLatLng
        {
            public double latitude;
            public double longitude;
        }

        #endregion

        #region Unity Lifecycle

        void Start()
        {
            InitializeComponents();

            // Wire recreation in code so it doesn't depend on Inspector event setup.
            if (driftCorrector != null)
                driftCorrector.OnExtremeDriftDetected.AddListener(RecreateAllAnchors);

            if (_proximity != null)
                _proximity.OnStatusUpdated += OnProximityDetectorStatusUpdated;

            if (_movement != null)
                _movement.OnStatusUpdated += OnMovementTrackerStatusUpdated;

            if (poiSpawner != null)
                poiSpawner.OnStatusUpdated += OnPOISpawnerStatusUpdated;

            StartCoroutine(InitializeGeospatialTracking());
        }

        void Update()
        {
            if (!isInitialized) return;

            CheckAndRefineAnchors();

            // Read the geospatial pose once per frame and share it, instead of each
            // sub-system querying the native AREarthManager pose independently.
            // Only valid while Earth is actively Tracking — otherwise leave it null.
            // (GeospatialPose is a struct, so pose.HasValue alone is NOT a validity
            // check: it would be true whenever earthManager != null.)
            GeospatialPose? pose =
                (earthManager != null && earthManager.EarthTrackingState == TrackingState.Tracking)
                    ? earthManager.CameraGeospatialPose
                    : (GeospatialPose?)null;

            UpdateTrackingState();
            UpdateVisibleAnchors();

            if (showAccuracyInUI && pose.HasValue)
                OnAccuracyUpdate?.Invoke((float)pose.Value.HorizontalAccuracy);

            if (_waypointNavActive && pose.HasValue)
                UpdateWaypointNavigation(pose.Value);
        }

        void OnDestroy()
        {
            ClearPath();

            if (driftCorrector != null)
                driftCorrector.OnExtremeDriftDetected.RemoveListener(RecreateAllAnchors);

            if (_proximity != null)
                _proximity.OnStatusUpdated -= OnProximityDetectorStatusUpdated;

            if (_movement != null)
                _movement.OnStatusUpdated -= OnMovementTrackerStatusUpdated;

            if (poiSpawner != null)
                poiSpawner.OnStatusUpdated -= OnPOISpawnerStatusUpdated;
        }

        private void OnProximityDetectorStatusUpdated(string status)
        {
            LogStatus($"[Proximity] {status}");
        }

        private void OnMovementTrackerStatusUpdated(string status)
        {
            LogStatus($"[Movement] {status}");
        }

        private void OnPOISpawnerStatusUpdated(string status)
        {
            LogStatus($"[POI Spawner] {status}");
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            if (driftCorrector != null)
            {
                driftCorrector.requiredAccuracy = requiredAccuracy;
            }
        }
#endif

        #endregion

        #region Initialization

        void InitializeComponents()
        {
            if (xrOrigin == null)
                xrOrigin = UnityEngine.Object.FindFirstObjectByType<XROrigin>();

            if (xrOrigin != null)
                arCamera = xrOrigin.Camera;

            if (poiSpawner != null && arCamera != null)
                poiSpawner.SetCamera(arCamera);

            if (earthManager == null)
                earthManager = UnityEngine.Object.FindFirstObjectByType<AREarthManager>();

            if (anchorManager == null)
                anchorManager = UnityEngine.Object.FindFirstObjectByType<ARAnchorManager>();

            if (arCoreExtensions == null)
                arCoreExtensions = UnityEngine.Object.FindFirstObjectByType<ARCoreExtensions>();

            // if (driftCorrector == null)
            // {
            //     driftCorrector = GetComponent<DriftCorrector>();
            //     if (driftCorrector == null)
            //         driftCorrector = gameObject.AddComponent<DriftCorrector>();
            //     LogStatus("DriftCorrector not assigned — added automatically.");
            // }

            if (driftCorrector != null)
            {
                driftCorrector.requiredAccuracy = requiredAccuracy;
            }

            if (xrOrigin == null) LogError("XROrigin not found! Add it to your scene.");
            if (arCamera == null) LogError("AR Camera not found in XROrigin!");
            if (earthManager == null) LogError("AREarthManager not found!");
            if (anchorManager == null) LogError("ARAnchorManager not found!");
            if (arCoreExtensions == null) LogError("ARCoreExtensions not found!");
            if (arrowPrefab == null) LogWarning("Arrow prefab not assigned!");
            if (terrainPrefab == null) LogWarning("Terrain prefab not assigned!");
            if (driftCorrector == null) LogWarning("DriftCorrector not assigned!");

            if (headingText == null)
            {
                GameObject canvasGo = GameObject.Find("Canvas");
                if (canvasGo != null)
                {
                    Transform arViewTrans = canvasGo.transform.Find("ARView");
                    if (arViewTrans != null)
                    {
                        Transform infoPanelTrans = arViewTrans.Find("InfoPanel");
                        if (infoPanelTrans != null)
                        {
                            Transform headingTrans = infoPanelTrans.Find("HeadingText");
                            if (headingTrans != null)
                            {
                                headingText = headingTrans.GetComponent<TextMeshProUGUI>();
                            }
                        }
                    }
                }
            }
        }

        IEnumerator InitializeGeospatialTracking()
        {
            LogStatus("Requesting location permission...");
            yield return RequestLocationPermission();

            if (!locationPermissionGranted)
            {
                LogError("Location permission denied. Geospatial features disabled.");
                yield break;
            }

            LogStatus("Location permission granted.");

            yield return new WaitUntil(() => ARSession.state == ARSessionState.SessionTracking);
            LogStatus("AR Session tracking.");

            LogStatus("Enabling Earth tracking...");
            yield return new WaitUntil(() => earthManager != null && earthManager.EarthState == EarthState.Enabled);
            LogStatus("Earth enabled.");

            LogStatus("Acquiring geospatial pose...");
            yield return new WaitUntil(() =>
                earthManager.EarthTrackingState == TrackingState.Tracking);

            LogStatus($"Waiting for GPS accuracy ≤ {requiredAccuracy}m (GPS-only mode — this may take 30–60s in open sky)...");
            yield return new WaitUntil(() =>
                earthManager.EarthTrackingState == TrackingState.Tracking &&
                earthManager.CameraGeospatialPose.HorizontalAccuracy <= requiredAccuracy);

            GeospatialPose acquiredPose = earthManager.CameraGeospatialPose;
            LogStatus($"Geospatial tracking acquired! Accuracy: {acquiredPose.HorizontalAccuracy:F2}m");

            isInitialized = true;
            OnTrackingStateChanged?.Invoke(true);

            if (autoCreateTestRoute)
                CreateTestRouteFromCurrentLocation();
        }

        #endregion

        #region Android Permissions

        IEnumerator RequestLocationPermission()
        {
            if (locationPermissionRequested) yield break;
            locationPermissionRequested = true;

            if (Permission.HasUserAuthorizedPermission(Permission.FineLocation))
            {
                locationPermissionGranted = true;
                yield break;
            }

            Permission.RequestUserPermission(Permission.FineLocation);

            float timeout = 30f;
            float elapsed = 0f;
            while (!Permission.HasUserAuthorizedPermission(Permission.FineLocation) && elapsed < timeout)
            {
                elapsed += Time.deltaTime;
                yield return null;
            }

            locationPermissionGranted = Permission.HasUserAuthorizedPermission(Permission.FineLocation);
            if (!locationPermissionGranted)
                LogError("User denied location permission or timeout occurred.");
        }

        public bool HasLocationPermission() => locationPermissionGranted;

        #endregion

        #region Public API - Path Creation

        /// <summary>
        /// Create navigation path from list of GPS coordinates.
        /// </summary>
        public void CreateNavigationPath(List<GPSCoordinate> routePoints)
        {
            if (!isInitialized)
            {
                LogWarning("Geospatial tracking not initialized yet. Cannot create path.");
                return;
            }

            if (routePoints == null || routePoints.Count < 2)
            {
                LogError("Route must contain at least 2 points.");
                return;
            }

            float currentAccuracy = GetCurrentAccuracy();
            if (currentAccuracy > requiredAccuracy)
                LogWarning($"GPS accuracy is marginal: {currentAccuracy:F2}m. Anchors may be offset. Recommended: <{requiredAccuracy:F2}m");

            LogStatus($"Creating navigation path with {routePoints.Count} route points (GPS accuracy: {currentAccuracy:F2}m)...");

            // A brand-new (non-recreation) route re-enables the one-shot accuracy
            // refinement. Recreation rebuilds (drift / refinement) keep it latched so
            // they can't loop — isRecreatingAnchors distinguishes the two.
            if (!isRecreatingAnchors)
                _hasRefinedAnchors = false;

            ClearPath();
            currentRoute = new List<GPSCoordinate>(routePoints);
            BuildAndPlaceAnchors(routePoints);
        }

        void BuildAndPlaceAnchors(List<GPSCoordinate> routePoints)
        {
            // Capture altitude ONCE for the entire route to prevent per-anchor jitter.
            // GPS altitude fluctuates ±5-15m between reads; using a single value
            // ensures all anchors in this batch sit at a consistent height.
            if (earthManager.EarthTrackingState == TrackingState.Tracking)
            {
                _routeBatchAltitude = earthManager.CameraGeospatialPose.Altitude - 1.5;
                LogStatus($"Batch altitude captured: {_routeBatchAltitude:F2}m");
            }

            List<GPSCoordinate> waypoints = GeoUtils.InterpolateWaypoints(routePoints, waypointIntervalMeters);
            LogStatus($"Generated {waypoints.Count} waypoints at {waypointIntervalMeters}m intervals.");
            _buildRoutine = StartCoroutine(CreateAnchorsSequentially(waypoints));

            StartWaypointNavigation();
        }

        // Adds the anchor to the path and registers it for drift monitoring immediately,
        // so anchors placed before a mid-route tracking loss are still monitored.
        void TrackPathAnchor(AnchorData anchorData)
        {
            // A terrain-resolve callback can still fire after the build was torn down
            // (ClearPath / a rebuild started). Don't resurrect a cleared path — destroy
            // the now-orphaned anchor instead of re-adding it to pathAnchors.
            if (!_isBuildingPath)
            {
                if (anchorData?.anchor != null) Destroy(anchorData.anchor.gameObject);
                if (anchorData?.visualObject != null) Destroy(anchorData.visualObject);
                return;
            }

            pathAnchors.Add(anchorData);
            if (driftCorrector != null && anchorData.anchor != null)
                driftCorrector.RegisterAnchor(anchorData.anchor, anchorData.originalCoordinate);
        }

        IEnumerator CreateAnchorsSequentially(List<GPSCoordinate> waypoints)
        {
            _isBuildingPath = true;
            try
            {
                int successCount = 0;
                int failedCount = 0;

                for (int i = 0; i < waypoints.Count - 1; i++)
                {
                    if (earthManager.EarthTrackingState != TrackingState.Tracking)
                    {
                        LogWarning($"Tracking lost at anchor {i + 1}/{waypoints.Count}. Stopping.");
                        yield break;
                    }

                    GPSCoordinate current = waypoints[i];
                    GPSCoordinate next = waypoints[i + 1];
                    Quaternion rotation = GeoUtils.CalculateBearingRotation(current, next);

                    if (useTerrainAnchors)
                    {
                        // Yield the enumerator directly (NOT StartCoroutine) so this child
                        // shares the parent's lifetime — StopCoroutine(_buildRoutine) in
                        // ClearPath then tears the child down too, instead of letting it
                        // complete and re-append anchors to a path we just cleared.
                        yield return CreateTerrainAnchorAsync(current, rotation, false,
                            anchorData =>
                            {
                                if (anchorData != null) { TrackPathAnchor(anchorData); successCount++; }
                                else failedCount++;
                            });
                    }
                    else
                    {
                        AnchorData anchorData = CreateStandardAnchor(current, rotation);
                        if (anchorData != null) { TrackPathAnchor(anchorData); successCount++; }
                        else failedCount++;
                    }

                    yield return _anchorPlacementDelay;
                }

                // Final destination marker
                GPSCoordinate destination = waypoints[waypoints.Count - 1];
                if (useTerrainAnchors)
                {
                    yield return CreateTerrainAnchorAsync(destination, Quaternion.identity, true,
                        anchorData =>
                        {
                            if (anchorData != null) { TrackPathAnchor(anchorData); successCount++; }
                            else failedCount++;
                        });
                }
                else
                {
                    AnchorData destAnchor = CreateStandardAnchor(destination, Quaternion.identity, true);
                    if (destAnchor != null) { TrackPathAnchor(destAnchor); successCount++; }
                    else failedCount++;
                }

                LogStatus($"✓ Navigation path created: {successCount} anchors placed, {failedCount} failed.");

                if (failedCount > waypoints.Count / 2)
                    LogError("⚠️ More than 50% of anchors failed. Check GPS signal quality.");
            }
            catch (System.Exception ex)
            {
                LogError($"Exception in CreateAnchorsSequentially: {ex.Message}\n{ex.StackTrace}");
            }
            finally
            {
                _isBuildingPath = false;
                _buildRoutine = null;
            }
        }

        [ContextMenu("Create Test Route")]
        public void CreateTestRouteFromCurrentLocation()
        {
            if (!isInitialized)
            {
                LogWarning("Cannot create test route: not initialized.");
                return;
            }

            GeospatialPose currentPose = earthManager.CameraGeospatialPose;
            double alt = currentPose.Altitude; // used only as fallback; terrain anchors override this

            // List<GPSCoordinate> testRoute = new List<GPSCoordinate>
            // {
            //     new GPSCoordinate(currentPose.Latitude, currentPose.Longitude, alt),
            //     new GPSCoordinate(33.69301, 73.05929, alt),
            //     new GPSCoordinate(33.69314, 73.05954, alt),
            //     new GPSCoordinate(33.69332, 73.05990, alt),
            //     new GPSCoordinate(33.69336, 73.05997, alt),
            //     new GPSCoordinate(33.69339, 73.06003, alt),
            //     new GPSCoordinate(33.69345, 73.06011, alt),
            //     new GPSCoordinate(33.69351, 73.06018, alt),
            //     new GPSCoordinate(33.69355, 73.06022, alt),
            //     new GPSCoordinate(33.69357, 73.06023, alt),
            //     new GPSCoordinate(33.69362, 73.06027, alt),
            //     new GPSCoordinate(33.69368, 73.06030, alt),
            //     new GPSCoordinate(33.69373, 73.06033, alt),
            //     new GPSCoordinate(33.69374, 73.06033, alt),
            //     new GPSCoordinate(33.69390, 73.06037, alt),
            //     new GPSCoordinate(33.69392, 73.06038, alt),
            //     new GPSCoordinate(33.69478, 73.06063, alt),
            //     new GPSCoordinate(33.69504, 73.06071, alt),
            //     new GPSCoordinate(33.69513, 73.06074, alt),
            //     new GPSCoordinate(33.69530, 73.06080, alt),
            //     new GPSCoordinate(33.69592, 73.06100, alt),
            //     new GPSCoordinate(33.69598, 73.06102, alt),
            //     new GPSCoordinate(33.69603, 73.06104, alt),
            //     new GPSCoordinate(33.69608, 73.06105, alt),
            //     new GPSCoordinate(33.69611, 73.06105, alt),
            //     new GPSCoordinate(33.69614, 73.06105, alt),
            //     new GPSCoordinate(33.69618, 73.06104, alt),
            //     new GPSCoordinate(33.69620, 73.06104, alt),
            //     new GPSCoordinate(33.69623, 73.06102, alt),
            // };

            List<GPSCoordinate> testRoute = new List<GPSCoordinate>{
        new(currentPose.Latitude, currentPose.Longitude, alt),
    new(33.69334, 73.05894, alt),
    new(33.69350, 73.05923, alt),
    new(33.69351, 73.05924, alt),
    new(33.69314, 73.05954, alt),
    new(33.69301, 73.05929, alt),};

            LogStatus("Creating test route...");
            CreateNavigationPath(testRoute);
        }

        [ContextMenu("Clear Path")]
        public void ClearPath()
        {
            // Stop any in-flight build so it can't keep appending anchors to the
            // list we're about to clear, racing with whatever rebuilds next.
            if (_buildRoutine != null) { StopCoroutine(_buildRoutine); _buildRoutine = null; }
            _isBuildingPath = false;

            foreach (var anchorData in pathAnchors)
            {
                if (anchorData.anchor != null) Destroy(anchorData.anchor.gameObject);
                if (anchorData.visualObject != null) Destroy(anchorData.visualObject);
                if (anchorData.driftDebugSphere != null) Destroy(anchorData.driftDebugSphere);
            }

            pathAnchors.Clear();
            currentRoute.Clear();

            // Force the next build to recapture its own batch altitude rather than
            // reusing this (now-cleared) route's value.
            _routeBatchAltitude = double.MinValue;

            if (driftCorrector != null) driftCorrector.ClearAnchors();

            if (poiSpawner != null) poiSpawner.DespawnCurrent();
            _waypointNavActive = false;
            if (wrongWayUI != null) wrongWayUI.SetActive(false);
            UpdateHeadingText("");

            LogStatus("Navigation path cleared.");
        }

        #endregion

        #region Anchor Creation

        /// <summary>
        /// Creates a Terrain Anchor — altitude is resolved from Google's elevation model.
        /// This is the correct anchor type for GPS-only regions (no VPS required).
        /// altitudeAboveTerrain = 0 places the anchor at ground level.
        /// </summary>
        IEnumerator CreateTerrainAnchorAsync(GPSCoordinate coordinate, Quaternion rotation,
            bool isDestination, System.Action<AnchorData> onComplete)
        {
            if (earthManager.EarthTrackingState != TrackingState.Tracking)
            {
                LogWarning($"Cannot create terrain anchor: tracking state is {earthManager.EarthTrackingState}");
                onComplete?.Invoke(null);
                yield break;
            }

            LogStatus($"Resolving terrain anchor at ({coordinate.latitude:F6}, {coordinate.longitude:F6})...");

            ResolveAnchorOnTerrainPromise promise =
                anchorManager.ResolveAnchorOnTerrainAsync(
                    coordinate.latitude,
                    coordinate.longitude,
                    0.0,            // altitudeAboveTerrain: 0 = ground level
                    rotation
                );

            // Wait for the async operation to complete
            while (promise.State == PromiseState.Pending)
            {
                yield return null;
            }

            var result = promise.Result;

            if (result.TerrainAnchorState != TerrainAnchorState.Success || result.Anchor == null)
            {
                LogWarning($"Terrain anchor failed ({result.TerrainAnchorState}), falling back to standard anchor.");
                AnchorData fallback = CreateStandardAnchor(coordinate, rotation, isDestination);
                onComplete?.Invoke(fallback);
                yield break;
            }

            LogStatus($"✓ Terrain anchor resolved at {coordinate}");

            // Use terrainPrefab for terrain anchors, fallback to arrowPrefab if not assigned
            GameObject prefabToSpawn = terrainPrefab != null ? terrainPrefab : arrowPrefab;
            GameObject visual = SpawnVisual(prefabToSpawn, result.Anchor.transform, isDestination);
            AnchorData anchorData = new AnchorData(result.Anchor, coordinate, visual);
            AttachDebugSphere(result.Anchor.transform, anchorData);

            onComplete?.Invoke(anchorData);
        }

        /// <summary>
        /// Standard geospatial anchor fallback (GPS altitude — less accurate vertically).
        /// Uses camera pose altitude instead of KML altitude to avoid floating/sinking.
        /// </summary>
        AnchorData CreateStandardAnchor(GPSCoordinate coordinate, Quaternion rotation, bool isDestination = false)
        {
            if (earthManager.EarthTrackingState != TrackingState.Tracking)
            {
                LogWarning($"Cannot create anchor: tracking state is {earthManager.EarthTrackingState}");
                return null;
            }

            // FIX: Use the batch altitude captured at route creation time.
            // This prevents per-anchor altitude jitter from GPS altitude fluctuations.
            // Falls back to live camera altitude only if no batch altitude was captured.
            double groundAltitude = _routeBatchAltitude > double.MinValue
                ? _routeBatchAltitude
                : earthManager.CameraGeospatialPose.Altitude - 1.5;

            LogStatus($"Creating standard anchor at ({coordinate.latitude:F6}, {coordinate.longitude:F6}), alt={groundAltitude:F1}m");

            ARGeospatialAnchor anchor = null;
            try
            {
                anchor = anchorManager.AddAnchor(
                    coordinate.latitude,
                    coordinate.longitude,
                    groundAltitude,
                    rotation
                );
            }
            catch (System.Exception e)
            {
                LogError($"AddAnchor failed: {e.Message}");
                return null;
            }

            if (anchor == null)
            {
                LogError($"Failed to create anchor at ({coordinate.latitude}, {coordinate.longitude})");
                return null;
            }

            LogStatus($"✓ Standard anchor created at {coordinate}");
            GameObject visual = SpawnVisual(arrowPrefab, anchor.transform, isDestination);
            AnchorData anchorData = new AnchorData(anchor, coordinate, visual);
            AttachDebugSphere(anchor.transform, anchorData);
            return anchorData;
        }

        GameObject SpawnVisual(GameObject prefab, Transform parent, bool isDestination)
        {
            if (prefab == null) return null;
            GameObject visual = Instantiate(prefab, parent);
            visual.transform.localPosition = Vector3.zero;
            visual.transform.localRotation = Quaternion.identity;
            if (isDestination) visual.transform.localScale *= 1.5f;
            return visual;
        }

        void AttachDebugSphere(Transform parent, AnchorData anchorData)
        {

            if (!showDriftDebugSpheres) return;
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);

            // Strip the auto-added collider so this debug primitive can't interfere
            // with AR raycasts / physics.
            var sphereCollider = sphere.GetComponent<Collider>();
            if (sphereCollider != null) Destroy(sphereCollider);

            sphere.transform.SetParent(parent);
            sphere.transform.localPosition = Vector3.zero;
            sphere.transform.localScale = Vector3.one * 0.3f;

            var renderer = sphere.GetComponent<Renderer>();
            if (driftDebugMaterial != null)
                renderer.sharedMaterial = driftDebugMaterial; // shared — no per-sphere material instance leak
            else
                renderer.material.color = Color.yellow;

            anchorData.driftDebugSphere = sphere;
        }

        #endregion

        #region Anchor Recreation

        public void RecreateAllAnchors()
        {
            if (currentRoute.Count == 0 || isRecreatingAnchors) return;
            isRecreatingAnchors = true;
            if (driftCorrector != null) driftCorrector.NotifyRecreatingAnchors();

            List<GPSCoordinate> routeCopy = new List<GPSCoordinate>(currentRoute);
            ClearPath();
            StartCoroutine(RecreateAnchorsAfterDelay(routeCopy));
        }

        IEnumerator RecreateAnchorsAfterDelay(List<GPSCoordinate> route)
        {
            yield return new WaitForSeconds(0.5f);
            CreateNavigationPath(route);
            yield return new WaitForSeconds(1f);
            isRecreatingAnchors = false;
            LogStatus("Anchor recreation complete.");
        }

        #endregion

        #region Runtime Updates

        void CheckAndRefineAnchors()
        {
            if (_hasRefinedAnchors || pathAnchors.Count == 0 || _isBuildingPath) return;
            if (earthManager.EarthTrackingState != TrackingState.Tracking) return;

            float currentAccuracy = GetCurrentAccuracy();
            if (currentAccuracy <= 5f)
            {
                LogStatus("Accuracy improved to 5m — refining anchor positions.");
                RecreateAllAnchors();
                _hasRefinedAnchors = true;
            }
        }
        void UpdateTrackingState()
        {
            if (earthManager == null) return;
            TrackingState currentState = earthManager.EarthTrackingState;
            if (currentState != TrackingState.Tracking)
            {
                LogWarning($"Tracking state: {currentState}");
                OnTrackingStateChanged?.Invoke(false);
            }
        }

        void UpdateVisibleAnchors()
        {
            if (!isInitialized || pathAnchors.Count == 0 || arCamera == null) return;

            foreach (var anchorData in pathAnchors)
            {
                if (anchorData.anchor == null) continue;
                float distance = Vector3.Distance(arCamera.transform.position, anchorData.anchor.transform.position);

                // FIX: Toggle visibility on the VISUAL CHILD only, not the anchor GameObject.
                // Calling SetActive on the anchor itself interferes with ARCore's internal
                // tracking of that anchor, contributing to position drift.
                if (anchorData.visualObject != null)
                    anchorData.visualObject.SetActive(distance < maxVisibleDistance);
            }
        }

        #endregion

        #region Public Getters

        public GPSCoordinate GetCurrentPosition()
        {
            if (!isInitialized || earthManager == null ||
                earthManager.EarthTrackingState != TrackingState.Tracking) return null;
            GeospatialPose pose = earthManager.CameraGeospatialPose;
            return new GPSCoordinate(pose.Latitude, pose.Longitude, pose.Altitude);
        }

        public float GetCurrentAccuracy()
        {
            if (!isInitialized || earthManager == null ||
                earthManager.EarthTrackingState != TrackingState.Tracking) return -1f;
            return (float)earthManager.CameraGeospatialPose.HorizontalAccuracy;
        }

        public bool IsTrackingReady() =>
            isInitialized && earthManager != null &&
            earthManager.EarthTrackingState == TrackingState.Tracking;

        public bool IsTracking() =>
        earthManager != null &&
        earthManager.EarthTrackingState == TrackingState.Tracking;

        public string GetTrackingStatusText()
        {
            if (!isInitialized) return "Initializing...";
            if (earthManager == null) return "Error: Earth Manager not found";

            bool tracking = earthManager.EarthTrackingState == TrackingState.Tracking;

            // Count anchors whose VISUAL is currently shown. Visibility is toggled on
            // visualObject (not the anchor GameObject), so testing anchor.activeSelf
            // would always be true. Manual loop avoids a per-call LINQ allocation.
            int validAnchors = 0;
            for (int i = 0; i < pathAnchors.Count; i++)
            {
                var a = pathAnchors[i];
                if (a.anchor != null && a.visualObject != null && a.visualObject.activeSelf)
                    validAnchors++;
            }

            string poseBlock;
            if (tracking)
            {
                GeospatialPose pose = earthManager.CameraGeospatialPose;
                poseBlock =
                    $"Accuracy: {pose.HorizontalAccuracy:F2}m (H) / {pose.VerticalAccuracy:F2}m (V)\n" +
                    $"Position: {pose.Latitude:F6}, {pose.Longitude:F6}\n" +
                    $"Altitude: {pose.Altitude:F1}m\n";
            }
            else
            {
                poseBlock = "Accuracy: --\nPosition: --\nAltitude: --\n";
            }

            return $"State: {earthManager.EarthTrackingState}\n" +
                   $"GPS Mode: No VPS (GPS-only)\n" +
                   poseBlock +
                   $"Anchors: {validAnchors}/{pathAnchors.Count} valid\n" +
                   $"Terrain Anchors: {(useTerrainAnchors ? "ON" : "OFF")}\n" +
                   $"Drift Correction: {(driftCorrector != null && driftCorrector.enableDriftCorrection ? "ON" : "OFF")}" +
                   (isRecreatingAnchors ? "\n⚠️ RECREATING ANCHORS..." : "");
        }

        public float GetDistanceToNearestWaypoint()
        {
            if (!isInitialized || pathAnchors.Count == 0 || arCamera == null) return -1f;

            float minDistance = float.MaxValue;
            foreach (var anchorData in pathAnchors)
            {
                // Only consider currently-visible waypoints. Visibility lives on
                // visualObject, not the anchor GameObject.
                if (anchorData.anchor == null ||
                    anchorData.visualObject == null || !anchorData.visualObject.activeSelf) continue;
                float d = Vector3.Distance(arCamera.transform.position, anchorData.anchor.transform.position);
                if (d < minDistance) minDistance = d;
            }
            return minDistance == float.MaxValue ? -1f : minDistance;
        }

        [ContextMenu("Force Drift Correction")]
        public void ForceDriftCorrection() => driftCorrector?.ForceDriftCorrection();

        [ContextMenu("Recreate All Anchors")]
        public void ForceRecreateAnchors() => RecreateAllAnchors();

        #endregion

        #region Logging

        void LogStatus(string message)
        {
            if (enableDebugLogs) Debug.Log($"[NavigationController] {message}");
            OnStatusUpdate?.Invoke(message);

            if (DebugText != null) DebugText.text = message;
        }

        void LogWarning(string message)
        {
            if (enableDebugLogs) Debug.LogWarning($"[NavigationController] {message}");
            OnStatusUpdate?.Invoke($"WARNING: {message}");
            if (DebugText != null) DebugText.text = message;
        }

        void LogError(string message)
        {
            Debug.LogError($"[NavigationController] {message}");
            OnStatusUpdate?.Invoke($"ERROR: {message}");
            if (DebugText != null) DebugText.text = message;
        }

        #endregion

        #region Waypoint Navigation

        private void StartWaypointNavigation()
        {
            if (currentRoute == null || currentRoute.Count < 2)
            {
                LogWarning("Route must have at least 2 waypoints for turn navigation.");
                _waypointNavActive = false;
                return;
            }

            _refLat = currentRoute[0].latitude;
            _refLon = currentRoute[0].longitude;
            _refAlt = currentRoute[0].altitude;

            if (wrongWayUI != null) wrongWayUI.SetActive(false);

            // Old samples were computed against the previous ENU reference origin
            // (_refLat/_refLon just changed above) — carrying them over would
            // blend two different local-space frames into a garbage heading.
            _movement.Reset();

            _currentWaypointIndex = ResolveStartWaypointIndex();
            PrepareTurnPOI();
            _waypointNavActive = true;
            LogStatus("Waypoint navigation started.");
        }

        // On a fresh route, start at waypoint 1. When resuming after anchor
        // recreation (drift correction / accuracy refinement mid-route), resume
        // at the nearest upcoming waypoint instead of restarting the whole route
        // and re-triggering wrong-way detection against the user's real position.
        private int ResolveStartWaypointIndex()
        {
            if (!isRecreatingAnchors || earthManager == null ||
                earthManager.EarthTrackingState != TrackingState.Tracking)
                return 1;

            var pose = earthManager.CameraGeospatialPose;
            Vector3 userLocal = GeoMath.GeoToLocal(
                pose.Latitude, pose.Longitude, pose.Altitude,
                _refLat, _refLon, _refAlt);
            Vector2 user = new Vector2(userLocal.x, userLocal.z);

            // Find the route segment the user is actually on (nearest by perpendicular
            // projection), then target that segment's END waypoint — i.e. the nearest
            // *upcoming* waypoint, never one already behind the user (which would
            // otherwise drive them backwards and trip wrong-way detection).
            int best = 1;
            float bestDistSq = float.MaxValue;
            for (int i = 1; i < currentRoute.Count; i++)
            {
                Vector3 aLocal = GeoMath.GeoToLocal(
                    currentRoute[i - 1].latitude, currentRoute[i - 1].longitude, currentRoute[i - 1].altitude,
                    _refLat, _refLon, _refAlt);
                Vector3 bLocal = GeoMath.GeoToLocal(
                    currentRoute[i].latitude, currentRoute[i].longitude, currentRoute[i].altitude,
                    _refLat, _refLon, _refAlt);
                Vector2 a = new Vector2(aLocal.x, aLocal.z);
                Vector2 b = new Vector2(bLocal.x, bLocal.z);

                Vector2 ab = b - a;
                float segLenSq = ab.sqrMagnitude;
                float t = segLenSq > 1e-6f ? Mathf.Clamp01(Vector2.Dot(user - a, ab) / segLenSq) : 0f;
                Vector2 proj = a + t * ab;
                float dSq = (user - proj).sqrMagnitude;
                if (dSq < bestDistSq) { bestDistSq = dSq; best = i; }
            }
            return best;
        }

        private void UpdateWaypointNavigation(GeospatialPose geoPose)
        {
            if (earthManager.EarthTrackingState != TrackingState.Tracking)
                return;

            float accuracy = (float)geoPose.HorizontalAccuracy;

            // --- Update user movement tracker ---
            UpdateUserMovement(geoPose, out Vector3 userLocal);

            // --- Proximity check ---
            if (_currentWaypointIndex >= currentRoute.Count) return;

            var wp = currentRoute[_currentWaypointIndex];
            float dist = (float)GeoMath.HaversineDistance(
                geoPose.Latitude, geoPose.Longitude,
                wp.latitude, wp.longitude);

            // --- Wrong-way detection ---
            UpdateWrongWayDetection(wp, userLocal);

            // --- Dynamic turn reclassification ---
            UpdateDynamicTurnReclassification(wp, dist);

            // --- Waypoint reached? ---
            UpdateWaypointProximity(dist, accuracy);

            // --- POI refinement ---
            UpdatePoiRefinement(wp);

        }


        private void UpdateUserMovement(GeospatialPose geoPose, out Vector3 userLocal)
        {
            userLocal = GeoMath.GeoToLocal(
                geoPose.Latitude, geoPose.Longitude, geoPose.Altitude,
                _refLat, _refLon, _refAlt);
            Vector2 userEN = new Vector2(userLocal.x, userLocal.z);
            _movement.Update(userEN, Time.time);
        }

        private void UpdateWrongWayDetection(GPSCoordinate wp, Vector3 userLocal)
        {
            if (!_movement.HasValidHeading) return;

            Vector3 wpLocal = GeoMath.GeoToLocal(
                wp.latitude, wp.longitude, wp.altitude,
                _refLat, _refLon, _refAlt);
            Vector2 dirToWaypoint = new Vector2(
                wpLocal.x - userLocal.x,
                wpLocal.z - userLocal.z);

            bool wrongWay = _wrongWay.Update(
                _movement.SmoothedDirection, _movement.Speed,
                dirToWaypoint, Time.deltaTime);

            if (wrongWayUI != null)
                wrongWayUI.SetActive(wrongWay);

            // Heading text must update regardless of whether the optional wrongWayUI
            // GameObject is assigned.
            UpdateHeadingText(wrongWay ? "Wrong Way!" : _displayedTurn.ToString());
        }

        private void UpdateDynamicTurnReclassification(GPSCoordinate wp, float dist)
        {
            if (!_poiActive
                || !_movement.HasValidHeading
                || _currentWaypointIndex >= currentRoute.Count - 1
                || dist >= 30f) // only reclassify when reasonably close
            {
                return;
            }

            Vector3 currLocal = GeoMath.GeoToLocal(
                wp.latitude, wp.longitude, wp.altitude,
                _refLat, _refLon, _refAlt);
            var nextWp = currentRoute[_currentWaypointIndex + 1];
            Vector3 nextLocal = GeoMath.GeoToLocal(
                nextWp.latitude, nextWp.longitude, nextWp.altitude,
                _refLat, _refLon, _refAlt);

            var (dynamicTurn, dynamicAngle) = TurnClassifier.ClassifyDynamic(
                _movement.SmoothedDirection, currLocal, nextLocal);

            // Update the POI if the turn direction changed — but require a
            // cooldown and a clear margin past the straight threshold so a
            // noisy heading near the classification boundary can't respawn
            // the POI (destroy/instantiate + anchor churn) every frame.
            if (dynamicTurn != _displayedTurn
                && dynamicTurn != TurnDirection.Straight
                && Time.time - _lastReclassifyTime > ReclassifyCooldown
                && Mathf.Abs(dynamicAngle) > 20f + ReclassifyMarginDeg)
            {
                _lastReclassifyTime = Time.time;
                LogStatus($"Turn reclassified: {_displayedTurn} → {dynamicTurn} " +
                          $"(angle: {dynamicAngle:F1}°)");
                _displayedTurn = dynamicTurn;

                UpdateHeadingText(dynamicTurn.ToString());

                // Re-spawn the POI with the corrected direction
                if (poiSpawner != null)
                    poiSpawner.SpawnTurnPOI(
                        wp.latitude, wp.longitude, wp.altitude, dynamicTurn);
            }
        }

        private void UpdateWaypointProximity(float dist, float accuracy)
        {
            if (_proximity.UpdateProximity(dist, accuracy))
            {
                OnWaypointReached();
            }
        }

        private void UpdatePoiRefinement(GPSCoordinate wp)
        {
            if (!_poiActive || poiSpawner == null) return;

            _refinementTimer += Time.deltaTime;
            if (_refinementTimer >= RefinementInterval)
            {
                _refinementTimer = 0f;
                poiSpawner.RefineAnchorPosition(
                    wp.latitude, wp.longitude, wp.altitude);
            }
        }

        private void OnWaypointReached()
        {
            LogStatus($"Reached waypoint {_currentWaypointIndex}");
            _currentWaypointIndex++;
            _wrongWay.Reset();

            if (_currentWaypointIndex >= currentRoute.Count)
            {
                LogStatus("🏁 Destination reached!");
                if (poiSpawner != null) poiSpawner.DespawnCurrent();
                _poiActive = false;
                if (wrongWayUI != null) wrongWayUI.SetActive(false);
                _waypointNavActive = false;
                UpdateHeadingText("Destination Reached");
                return;
            }

            PrepareTurnPOI();
        }

        private void PrepareTurnPOI()
        {
            _proximity.Reset();
            _wrongWay.Reset();
            if (poiSpawner != null) poiSpawner.DespawnCurrent();
            _poiActive = false;
            _refinementTimer = 0f;

            // Only show turn POIs at vertices with actual turns (not the final destination)
            if (_currentWaypointIndex <= 0 || _currentWaypointIndex >= currentRoute.Count - 1)
            {
                if (_currentWaypointIndex > 0)
                {
                    UpdateHeadingText(TurnDirection.Straight.ToString());
                }
                return;
            }

            // --- Static classification (route geometry) ---
            var prev = currentRoute[_currentWaypointIndex - 1];
            var curr = currentRoute[_currentWaypointIndex];
            var next = currentRoute[_currentWaypointIndex + 1];

            Vector3 a = GeoMath.GeoToLocal(
                prev.latitude, prev.longitude, prev.altitude,
                _refLat, _refLon, _refAlt);
            Vector3 b = GeoMath.GeoToLocal(
                curr.latitude, curr.longitude, curr.altitude,
                _refLat, _refLon, _refAlt);
            Vector3 c = GeoMath.GeoToLocal(
                next.latitude, next.longitude, next.altitude,
                _refLat, _refLon, _refAlt);

            var (turn, angle) = TurnClassifier.Classify(a, b, c);
            _displayedTurn = turn;

            UpdateHeadingText(turn.ToString() + " " + angle.ToString("F1") + "°");

            if (turn == TurnDirection.Straight)
            {
                // Optionally log that we are skipping this POI because it's straight
                return;
            }

            if (poiSpawner != null)
            {
                poiSpawner.SpawnTurnPOI(
                    curr.latitude, curr.longitude, curr.altitude, turn);
                _poiActive = true;
            }
        }

        private void UpdateHeadingText(string text)
        {
            if (headingText != null)
            {
                headingText.text = text;
            }
        }

        #endregion
    }

    /// <summary>
    /// GPS coordinate data structure — uses double precision throughout.
    /// </summary>
    [System.Serializable]
    public class GPSCoordinate
    {
        public double latitude;
        public double longitude;
        public double altitude;

        public GPSCoordinate() { }

        public GPSCoordinate(double lat, double lon, double alt = 0)
        {
            latitude = lat;
            longitude = lon;
            altitude = alt;
        }

        public override string ToString() => $"({latitude:F6}, {longitude:F6}, {altitude:F1}m)";
    }
}
