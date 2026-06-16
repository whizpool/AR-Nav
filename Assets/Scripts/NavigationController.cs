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

    /// <summary>
    /// Enhanced Navigation Controller with drift correction and anchor stabilization.
    /// Fixed for GPS-only regions (no VPS): double precision math, terrain anchors, Roads API snap.
    /// Compatible with Unity 6.3 LTS - uses XROrigin instead of deprecated ARSessionOrigin.
    /// </summary>
    public class NavigationController : MonoBehaviour

    {
        public Text DebugText;

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
        [Range(1f, 20f)]
        public float requiredAccuracy = 10f;

        [Header("Roads API (Snap to Road)")]
        [Tooltip("Your Google Cloud API key with Roads API enabled")]
        public string googleApiKey = "";

        [Tooltip("Snap route coordinates to road centerline before placing anchors")]
        public bool snapToRoads = true;

        [Header("Drift Correction Settings")]
        [Tooltip("Enable automatic drift correction")]
        public bool enableDriftCorrection = true;

        [Tooltip("Time between drift correction updates (seconds)")]
        [Range(5f, 30f)]
        public float driftCorrectionInterval = 10f;

        [Tooltip("Recreate anchors if drift exceeds this threshold (meters)")]
        [Range(3f, 15f)]
        public float recreateAnchorThreshold = 5f;

        [Header("Anchor Settings")]
        [Tooltip("Use terrain anchors for altitude stability (recommended for GPS-only regions, no VPS needed)")]
        public bool useTerrainAnchors = true;

        // Rooftop anchors require VPS — disabled in GPS-only regions
        // public bool useRooftopAnchors = false;

        [Tooltip("Anchor lifetime before automatic refresh (0 = infinite)")]
        [Range(0f, 300f)]
        public float anchorLifetimeSeconds = 0f;

        [Header("Debug Settings")]
        public bool enableDebugLogs = true;
        public bool showAccuracyInUI = true;
        public bool showDriftDebugSpheres = false;
        public Material driftDebugMaterial;

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
        private float lastDriftCheckTime;
        private bool isRecreatingAnchors = false;

        // Permission tracking
        private bool locationPermissionGranted = false;
        private bool locationPermissionRequested = false;

        private bool _hasRefinedAnchors = false;

        // Tracking state
        private bool isInitialized = false;
        private GeospatialPose lastKnownPose;
        private GeospatialPose initialPose;

        // UI callback delegates
        public System.Action<string> OnStatusUpdate;
        public System.Action<bool> OnTrackingStateChanged;
        public System.Action<float> OnAccuracyUpdate;
        public System.Action<float> OnDriftDetected;

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
            StartCoroutine(InitializeGeospatialTracking());
        }

        void Update()
        {
            if (!isInitialized) return;

            CheckAndRefineAnchors();

            UpdateTrackingState();
            UpdateVisibleAnchors();

            if (enableDriftCorrection && Time.time - lastDriftCheckTime > driftCorrectionInterval)
            {
                PerformDriftCorrection();
                lastDriftCheckTime = Time.time;
            }

            if (anchorLifetimeSeconds > 0)
                CheckAnchorLifetime();

            if (showAccuracyInUI && earthManager != null)
                OnAccuracyUpdate?.Invoke((float)earthManager.CameraGeospatialPose.HorizontalAccuracy);
        }

        void OnDestroy()
        {
            ClearPath();
        }

        #endregion

        #region Initialization

        void InitializeComponents()
        {
            if (xrOrigin == null)
                xrOrigin = UnityEngine.Object.FindFirstObjectByType<XROrigin>();

            if (xrOrigin != null)
                arCamera = xrOrigin.Camera;

            if (earthManager == null)
                earthManager = UnityEngine.Object.FindFirstObjectByType<AREarthManager>();

            if (anchorManager == null)
                anchorManager = UnityEngine.Object.FindFirstObjectByType<ARAnchorManager>();

            if (arCoreExtensions == null)
                arCoreExtensions = UnityEngine.Object.FindFirstObjectByType<ARCoreExtensions>();

            if (xrOrigin == null) LogError("XROrigin not found! Add it to your scene.");
            if (arCamera == null) LogError("AR Camera not found in XROrigin!");
            if (earthManager == null) LogError("AREarthManager not found!");
            if (anchorManager == null) LogError("ARAnchorManager not found!");
            if (arCoreExtensions == null) LogError("ARCoreExtensions not found!");
            if (arrowPrefab == null) LogWarning("Arrow prefab not assigned!");
            if (terrainPrefab == null) LogWarning("Terrain prefab not assigned!");
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
                earthManager.CameraGeospatialPose.HorizontalAccuracy <= requiredAccuracy);

            initialPose = earthManager.CameraGeospatialPose;
            LogStatus($"Geospatial tracking acquired! Accuracy: {initialPose.HorizontalAccuracy:F2}m");

            isInitialized = true;
            lastDriftCheckTime = Time.time;
            OnTrackingStateChanged?.Invoke(true);

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
        /// If snapToRoads is enabled, coordinates are first snapped to road centerline via Roads API.
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

            ClearPath();
            currentRoute = new List<GPSCoordinate>(routePoints);

            if (snapToRoads && !string.IsNullOrEmpty(googleApiKey))
            {
                StartCoroutine(SnapAndCreatePath(routePoints));
            }
            else
            {
                if (snapToRoads)
                    LogWarning("Roads API key not set — skipping snap to roads. Set googleApiKey in Inspector.");
                BuildAndPlaceAnchors(routePoints);
            }
        }

        /// <summary>
        /// Snap route coordinates to road centerline using Google Roads API, then place anchors.
        /// This is the key fix for routes appearing beside the road in GPS-only regions.
        /// </summary>
        IEnumerator SnapAndCreatePath(List<GPSCoordinate> rawPoints)
        {
            LogStatus("Snapping coordinates to road centerline...");

            // Roads API accepts max 100 points per request; chunk if needed
            List<GPSCoordinate> snappedPoints = new List<GPSCoordinate>();
            int chunkSize = 100;

            for (int start = 0; start < rawPoints.Count; start += chunkSize)
            {
                int count = Mathf.Min(chunkSize, rawPoints.Count - start);
                List<GPSCoordinate> chunk = rawPoints.GetRange(start, count);

                // Build path string: "lat,lng|lat,lng|..."
                string path = string.Join("|", chunk.Select(p => $"{p.latitude},{p.longitude}"));
                string url = $"https://roads.googleapis.com/v1/snapToRoads" +
                             $"?path={UnityWebRequest.EscapeURL(path)}" +
                             $"&interpolate=true" +
                             $"&key={googleApiKey}";

                using var request = UnityWebRequest.Get(url);
                yield return request.SendWebRequest();

                if (request.result != UnityWebRequest.Result.Success)
                {
                    LogWarning($"Roads API failed: {request.error}. Proceeding with raw coordinates.");
                    BuildAndPlaceAnchors(rawPoints);
                    yield break;
                }

                SnapToRoadsResponse response = null;
                try
                {
                    response = JsonUtility.FromJson<SnapToRoadsResponse>(request.downloadHandler.text);
                }
                catch (System.Exception e)
                {
                    LogWarning($"Roads API parse error: {e.Message}. Proceeding with raw coordinates.");
                    BuildAndPlaceAnchors(rawPoints);
                    yield break;
                }

                if (response?.snappedPoints == null || response.snappedPoints.Count == 0)
                {
                    LogWarning("Roads API returned empty result. Proceeding with raw coordinates.");
                    BuildAndPlaceAnchors(rawPoints);
                    yield break;
                }

                // Use camera altitude for all snapped points — never trust KML/My Maps altitude
                double cameraAltitude = earthManager.CameraGeospatialPose.Altitude;
                foreach (var sp in response.snappedPoints)
                {
                    snappedPoints.Add(new GPSCoordinate(
                        sp.location.latitude,
                        sp.location.longitude,
                        cameraAltitude // overwritten below by terrain anchor anyway
                    ));
                }
            }

            LogStatus($"Roads API: {rawPoints.Count} raw points → {snappedPoints.Count} snapped points on road centerline.");
            BuildAndPlaceAnchors(snappedPoints);
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

            List<GPSCoordinate> waypoints = InterpolateWaypoints(routePoints, waypointIntervalMeters);
            LogStatus($"Generated {waypoints.Count} waypoints at {waypointIntervalMeters}m intervals.");
            StartCoroutine(CreateAnchorsSequentially(waypoints));
        }

        IEnumerator CreateAnchorsSequentially(List<GPSCoordinate> waypoints)
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
                Quaternion rotation = CalculateBearingRotation(current, next);

                if (useTerrainAnchors)
                {
                    yield return StartCoroutine(CreateTerrainAnchorAsync(current, rotation, false,
                        anchorData =>
                        {
                            if (anchorData != null) { pathAnchors.Add(anchorData); successCount++; }
                            else failedCount++;
                        }));
                }
                else
                {
                    AnchorData anchorData = CreateStandardAnchor(current, rotation);
                    if (anchorData != null) { pathAnchors.Add(anchorData); successCount++; }
                    else failedCount++;
                }

                yield return new WaitForSeconds(0.2f);
            }

            // Final destination marker
            GPSCoordinate destination = waypoints[waypoints.Count - 1];
            if (useTerrainAnchors)
            {
                yield return StartCoroutine(CreateTerrainAnchorAsync(destination, Quaternion.identity, true,
                    anchorData =>
                    {
                        if (anchorData != null) { pathAnchors.Add(anchorData); successCount++; }
                        else failedCount++;
                    }));
            }
            else
            {
                AnchorData destAnchor = CreateStandardAnchor(destination, Quaternion.identity, true);
                if (destAnchor != null) { pathAnchors.Add(destAnchor); successCount++; }
                else failedCount++;
            }

            LogStatus($"✓ Navigation path created: {successCount} anchors placed, {failedCount} failed.");

            if (failedCount > waypoints.Count / 2)
                LogError("⚠️ More than 50% of anchors failed. Check GPS signal quality.");
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

            List<GPSCoordinate> testRoute = new List<GPSCoordinate>
{
    new(currentPose.Latitude, currentPose.Longitude, alt),
    new(33.69301, 73.05929, alt),
    new GPSCoordinate(33.69314, 73.05954, alt),
    new GPSCoordinate(33.69351, 73.05924, alt),
    new GPSCoordinate(33.69350, 73.05923, alt),
    new GPSCoordinate(33.69334, 73.05894, alt),
};

            LogStatus("Creating test route...");
            CreateNavigationPath(testRoute);
        }

        [ContextMenu("Clear Path")]
        public void ClearPath()
        {
            foreach (var anchorData in pathAnchors)
            {
                if (anchorData.anchor != null) Destroy(anchorData.anchor.gameObject);
                if (anchorData.visualObject != null) Destroy(anchorData.visualObject);
                if (anchorData.driftDebugSphere != null) Destroy(anchorData.driftDebugSphere);
            }

            pathAnchors.Clear();
            currentRoute.Clear();
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
            yield return promise;

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
            sphere.transform.SetParent(parent);
            sphere.transform.localPosition = Vector3.zero;
            sphere.transform.localScale = Vector3.one * 0.3f;
            var renderer = sphere.GetComponent<Renderer>();
            renderer.material.color = driftDebugMaterial != null ? Color.yellow : Color.yellow;
            if (driftDebugMaterial != null) renderer.material = driftDebugMaterial;
            anchorData.driftDebugSphere = sphere;
        }

        #endregion

        #region Drift Correction

        void PerformDriftCorrection()
        {
            if (pathAnchors.Count == 0 || arCamera == null || isRecreatingAnchors) return;
            if (earthManager.EarthTrackingState != TrackingState.Tracking) return;

            GeospatialPose currentPose = earthManager.CameraGeospatialPose;

            if (currentPose.HorizontalAccuracy > requiredAccuracy * 2)
            {
                LogWarning($"Skipping drift correction — poor GPS accuracy: {currentPose.HorizontalAccuracy:F2}m");
                return;
            }

            int checkedAnchors = 0;
            float totalDrift = 0f;

            foreach (var anchorData in pathAnchors)
            {
                if (anchorData.anchor == null) continue;

                // FIXED: Extract Pose from Transform before passing to Convert()
                Pose anchorWorldPose = new Pose(
                    anchorData.anchor.transform.position,
                    anchorData.anchor.transform.rotation
                );
                GeospatialPose anchorPose = earthManager.Convert(anchorWorldPose);

                double drift = CalculateDistance(
                    new GPSCoordinate(anchorPose.Latitude, anchorPose.Longitude, anchorPose.Altitude),
                    anchorData.originalCoordinate);

                if (drift > 1.0 && drift < 50.0)
                {
                    totalDrift += (float)drift;
                    checkedAnchors++;
                }
            }

            if (checkedAnchors == 0) return;

            float averageDrift = totalDrift / checkedAnchors;

            if (averageDrift > 0.5f)
            {
                LogWarning($"Drift detected: {averageDrift:F2}m across {checkedAnchors} anchors");
                OnDriftDetected?.Invoke(averageDrift);
            }

            // FIX: Only recreate anchors for EXTREME drift (>15m) indicating a major GPS jump.
            // Moderate drift (< 15m) is normal in GPS-only mode and ARCore's internal
            // world model will self-correct. Frequent recreation causes visible anchor
            // jumps — the exact "moving backward" symptom the user reports.
            const float extremeDriftThreshold = 15f;
            if (averageDrift > extremeDriftThreshold)
            {
                LogWarning($"Extreme drift ({averageDrift:F2}m > {extremeDriftThreshold}m). Recreating anchors...");
                RecreateAllAnchors();
            }
            else if (averageDrift > recreateAnchorThreshold)
            {
                LogWarning($"Moderate drift ({averageDrift:F2}m) — NOT recreating anchors. " +
                           $"ARCore will self-correct. Threshold for recreation: {extremeDriftThreshold}m.");
            }
        }

        void RecreateAllAnchors()
        {
            if (currentRoute.Count == 0 || isRecreatingAnchors) return;
            isRecreatingAnchors = true;
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

        void CheckAnchorLifetime()
        {
            foreach (var anchorData in pathAnchors)
            {
                if (Time.time - anchorData.creationTime > anchorLifetimeSeconds)
                {
                    LogStatus("Anchors expired. Recreating...");
                    RecreateAllAnchors();
                    return;
                }
            }
        }

        #endregion

        #region Coordinate Utilities

        /// <summary>
        /// Interpolates waypoints along a route using full double precision.
        /// FIX: Replaced float Lerp (7 sig figs) with manual double interpolation (15 sig figs).
        /// Float precision alone causes 2–5m coordinate errors at GPS scale.
        /// </summary>
        List<GPSCoordinate> InterpolateWaypoints(List<GPSCoordinate> points, float intervalMeters)
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
        Quaternion CalculateBearingRotation(GPSCoordinate from, GPSCoordinate to)
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
        double CalculateDistance(GPSCoordinate from, GPSCoordinate to)
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

        GPSCoordinate OffsetCoordinate(GeospatialPose origin, double northMeters, double eastMeters)
        {
            double latOffset = northMeters / 111320.0;
            double lonOffset = eastMeters / (111320.0 * System.Math.Cos(origin.Latitude * System.Math.PI / 180.0));
            return new GPSCoordinate(origin.Latitude + latOffset, origin.Longitude + lonOffset, origin.Altitude);
        }

        #endregion

        #region Runtime Updates

        void CheckAndRefineAnchors()
        {
            if (_hasRefinedAnchors || pathAnchors.Count == 0) return;
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
            lastKnownPose = earthManager.CameraGeospatialPose;
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
            if (!isInitialized || earthManager == null) return null;
            GeospatialPose pose = earthManager.CameraGeospatialPose;
            return new GPSCoordinate(pose.Latitude, pose.Longitude, pose.Altitude);
        }

        public float GetCurrentAccuracy()
        {
            if (!isInitialized || earthManager == null) return -1f;
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

            GeospatialPose pose = earthManager.CameraGeospatialPose;
            int validAnchors = pathAnchors.Count(a => a.anchor != null && a.anchor.gameObject.activeSelf);

            return $"State: {earthManager.EarthTrackingState}\n" +
                   $"GPS Mode: No VPS (GPS-only)\n" +
                   $"Accuracy: {pose.HorizontalAccuracy:F2}m (H) / {pose.VerticalAccuracy:F2}m (V)\n" +
                   $"Position: {pose.Latitude:F6}, {pose.Longitude:F6}\n" +
                   $"Altitude: {pose.Altitude:F1}m\n" +
                   $"Anchors: {validAnchors}/{pathAnchors.Count} valid\n" +
                   $"Terrain Anchors: {(useTerrainAnchors ? "ON" : "OFF")}\n" +
                   $"Snap to Roads: {(snapToRoads ? "ON" : "OFF")}\n" +
                   $"Drift Correction: {(enableDriftCorrection ? "ON" : "OFF")}" +
                   (isRecreatingAnchors ? "\n⚠️ RECREATING ANCHORS..." : "");
        }

        public float GetDistanceToNearestWaypoint()
        {
            if (!isInitialized || pathAnchors.Count == 0 || arCamera == null) return -1f;

            float minDistance = float.MaxValue;
            foreach (var anchorData in pathAnchors)
            {
                if (anchorData.anchor == null || !anchorData.anchor.gameObject.activeSelf) continue;
                float d = Vector3.Distance(arCamera.transform.position, anchorData.anchor.transform.position);
                if (d < minDistance) minDistance = d;
            }
            return minDistance == float.MaxValue ? -1f : minDistance;
        }

        [ContextMenu("Force Drift Correction")]
        public void ForceDriftCorrection() => PerformDriftCorrection();

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
