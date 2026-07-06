namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using System.Collections;
    using System.Collections.Generic;
    using UnityEngine;
    using UnityEngine.UI;
    using TMPro;
    using Google.XR.ARCoreExtensions;
    using UnityEngine.XR.ARFoundation;
    using UnityEngine.XR.ARSubsystems;

    /// <summary>
    /// Simplified PoiManager:
    ///   • Only supports a predefined list of POIs.
    ///   • Places AR anchors for each POI (Terrain or Standard fallback).
    ///   • Billboards the 3D visuals so they always face the camera on the Y-axis.
    ///   • Controls the WorldSpacePanelUI to display details on proximity.
    /// </summary>
    public class PoiManager : MonoBehaviour
    {
        #region Inspector Fields
        [Header("Required References")]
        public NavigationController navController;
        public ARAnchorManager anchorManager;

        [Header("3D Anchor Visual")]
        public bool billboardPrefabs = true;

        [Header("Pre-defined POIs")]
        public List<PointOfInterest> pointsOfInterest = new List<PointOfInterest>();

        [Header("Default POI UI (Prefab or Scene Instance)")]
        public WorldSpacePanelUI worldSpacePanel;


        [Header("Radius UI")]
        public Slider radiusSlider;
        public TextMeshProUGUI radiusValueText;
        [Range(5f, 200f)]
        public float globalTriggerRadius = 30f;

        [Header("Behaviour")]
        [Range(0.1f, 2f)]
        public float checkIntervalSeconds = 0.5f;

        [Header("Debug")]
        public bool enableLogs = true;
        public bool showRadiusGizmos = true;
        #endregion

        #region Data Structures
        private class PoiRecord
        {
            public PointOfInterest poi;
            public ARGeospatialAnchor anchor;
            public GameObject visual3d;
            public WorldSpacePanelUI panelUI;
        }
        #endregion

        #region Private State
        private List<PoiRecord> records = new List<PoiRecord>();
        private List<PoiRecord> nearbyRecords = new List<PoiRecord>();
        private Camera arCamera;
        private bool initialized;
        private Coroutine proximityLoopCoroutine;
        #endregion


        #region Unity Methods
        void Start()
        {
            if (navController == null)
            {
                Debug.LogError("[PoiManager] navController not assigned.");
                return;
            }

            if (anchorManager == null)
                anchorManager = FindFirstObjectByType<ARAnchorManager>();

            foreach (var poi in pointsOfInterest)
                records.Add(new PoiRecord { poi = poi });

            if (radiusSlider != null)
            {
                radiusSlider.onValueChanged.AddListener((val) =>
                {
                    globalTriggerRadius = val;
                    if (radiusValueText != null)
                    {
                        radiusValueText.text = $"{val:F0}m";
                    }
                });
                radiusSlider.value = globalTriggerRadius;
            }

            StartCoroutine(WaitForTrackingThenInit());
        }

        void Update()
        {
            if (!initialized) return;

            if (billboardPrefabs) BillboardNearbyVisuals();
        }

        void OnDestroy()
        {
            if (proximityLoopCoroutine != null)
            {
                StopCoroutine(proximityLoopCoroutine);
                proximityLoopCoroutine = null;
            }

            foreach (var r in records)
            {
                if (r.visual3d != null) Destroy(r.visual3d);
                if (r.anchor != null) Destroy(r.anchor.gameObject);
            }
            records.Clear();
        }

        IEnumerator WaitForTrackingThenInit()
        {
            while (!navController.IsTrackingReady())
            {
                yield return null;
            }

            arCamera = navController.xrOrigin != null
                ? navController.xrOrigin.Camera
                : Camera.main;

            if (worldSpacePanel != null)
            {
                worldSpacePanel.gameObject.SetActive(false);
            }

            foreach (var record in records)
                yield return StartCoroutine(PlaceAnchor(record));

            initialized = true;

            if (proximityLoopCoroutine != null) StopCoroutine(proximityLoopCoroutine);
            proximityLoopCoroutine = StartCoroutine(ProximityCheckLoop());
        }

        IEnumerator ProximityCheckLoop()
        {
            var wait = new WaitForSeconds(checkIntervalSeconds);

            while (true)
            {
                if (arCamera != null)
                {
                    Vector3 cameraPos = arCamera.transform.position;
                    nearbyRecords.Clear();

                    foreach (var record in records)
                    {
                        if (record.anchor == null || record.visual3d == null) continue;

                        float dist = Vector3.Distance(cameraPos, record.anchor.transform.position);

                        // If POI is within global trigger radius, activate and track it
                        if (dist <= globalTriggerRadius)
                        {
                            if (!record.visual3d.activeSelf)
                            {
                                record.visual3d.SetActive(true);
                            }

                            nearbyRecords.Add(record);

                            // Update UI text at throttled intervals instead of every frame
                            if (record.panelUI != null)
                            {
                                record.panelUI.SetDistance($"{dist:F1} m");
                            }
                        }
                        else
                        {
                            // Deactivate objects that are too far away
                            if (record.visual3d.activeSelf)
                            {
                                record.visual3d.SetActive(false);
                            }
                        }
                    }
                }

                yield return wait;
            }
        }

        #endregion

        #region Anchor Placement
        IEnumerator PlaceAnchor(PoiRecord record)
        {
            var poi = record.poi;
            var rotation = Quaternion.Euler(0, poi.yRotationOverride, 0);

            // Attempt terrain anchor (altitude from Google elevation model)
            var promise = anchorManager.ResolveAnchorOnTerrainAsync(
                poi.latitude, poi.longitude, poi.heightAboveTerrain, rotation);

            yield return promise;

            if (promise.Result.TerrainAnchorState == TerrainAnchorState.Success
                && promise.Result.Anchor != null)
            {
                FinalizeAnchor(record, promise.Result.Anchor);
                yield break;
            }

            // Fallback to standard anchor
            var pose = navController.earthManager.CameraGeospatialPose;
            double altitude = poi.altitudeOverride > 0
                ? poi.altitudeOverride
                : pose.Altitude - 1.3 + poi.heightAboveTerrain;

            var anchor = anchorManager.AddAnchor(poi.latitude, poi.longitude, altitude, rotation);
            if (anchor != null)
            {
                FinalizeAnchor(record, anchor);
            }
        }

        void FinalizeAnchor(PoiRecord record, ARGeospatialAnchor anchor)
        {
            record.anchor = anchor;

            if (record.poi.poiPrefab != null)
            {
                record.visual3d = Instantiate(record.poi.poiPrefab, anchor.transform);
                record.visual3d.transform.localPosition = Vector3.zero;
                record.visual3d.transform.localRotation = Quaternion.identity;
                record.visual3d.SetActive(false);
            }
            else if (worldSpacePanel != null)
            {
                var panelInstance = Instantiate(worldSpacePanel, anchor.transform);
                panelInstance.transform.localPosition = Vector3.up * 1.8f;
                panelInstance.transform.localRotation = Quaternion.identity;

                // Ensure active so UIDocument elements are instantiated and resolved
                panelInstance.gameObject.SetActive(true);

                panelInstance.SetTitle(record.poi.title);
                panelInstance.SetDescription(record.poi.description);

                panelInstance.OnDismissClicked += () =>
                {
                    record.poi.lastDismissedTime = Time.time;
                    panelInstance.gameObject.SetActive(false);
                };

                record.visual3d = panelInstance.gameObject;
                record.panelUI = panelInstance;
            }
        }
        #endregion

        #region Visual Helpers
        void BillboardNearbyVisuals()
        {
            if (arCamera == null) return;
            foreach (var r in nearbyRecords)
            {
                if (r.visual3d == null) continue;
                Vector3 dir = r.visual3d.transform.position - arCamera.transform.position;
                dir.y = 0; // Rotate only on the Y axis
                if (dir.sqrMagnitude > 0.001f)
                    r.visual3d.transform.rotation = Quaternion.LookRotation(dir);
            }
        }
        #endregion

    }
}
