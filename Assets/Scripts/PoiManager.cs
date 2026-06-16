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
    /// </summary>
    public class PoiManager : MonoBehaviour
    {
        [Header("Required References")]
        public NavigationController navController;
        public ARAnchorManager anchorManager;

        [Header("3D Anchor Visual")]
        public GameObject defaultPoiPrefab;
        public bool billboardPrefabs = true;

        [Header("Pre-defined POIs")]
        public List<PointOfInterest> pointsOfInterest = new List<PointOfInterest>();

        // ── Preserved UI & Behaviour Fields (To prevent Inspector reference loss) ──

        [Header("Popup UI")]
        public GameObject poiPopupPanel;
        public TextMeshProUGUI popupTitle;
        public TextMeshProUGUI popupDescription;
        public TextMeshProUGUI popupDistance;
        public Button dismissButton;

        [Header("Spawn Button")]
        public Button spawnButton;
        public string autoSpawnTitle = "My Location";
        [TextArea(2, 3)]
        public string autoSpawnDescription = "Marked from current position.";
        [Range(0f, 5f)]
        public float autoSpawnHeight = 0f;

        [Header("Radius UI")]
        public Slider radiusSlider;
        public TextMeshProUGUI radiusValueText;
        [Range(5f, 200f)]
        public float globalTriggerRadius = 30f;

        [Header("Behaviour")]
        [Range(0.1f, 2f)]
        public float checkIntervalSeconds = 0.5f;

        [Header("Debug")]
        public bool enableLogs    = true;
        public bool showRadiusGizmos = true;

        private class PoiRecord
        {
            public PointOfInterest poi;
            public ARGeospatialAnchor anchor;
            public GameObject visual3d;
        }

        private List<PoiRecord> records = new List<PoiRecord>();
        private Camera arCamera;
        private bool initialized;

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

            StartCoroutine(WaitForTrackingThenInit());
        }

        void Update()
        {
            if (!initialized) return;

            if (billboardPrefabs) BillboardAllVisuals();
        }

        void OnDestroy()
        {
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

            foreach (var record in records)
                yield return StartCoroutine(PlaceAnchor(record));

            initialized = true;
        }

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
            var prefab = record.poi.poiPrefab != null ? record.poi.poiPrefab : defaultPoiPrefab;
            if (prefab != null)
            {
                record.visual3d = Instantiate(prefab, anchor.transform);
                record.visual3d.transform.localPosition = Vector3.zero;
                record.visual3d.transform.localRotation = Quaternion.identity;
            }
        }

        void BillboardAllVisuals()
        {
            if (arCamera == null) return;
            foreach (var r in records)
            {
                if (r.visual3d == null) continue;
                Vector3 dir = r.visual3d.transform.position - arCamera.transform.position;
                dir.y = 0; // Rotate only on the Y axis
                if (dir.sqrMagnitude > 0.001f)
                    r.visual3d.transform.rotation = Quaternion.LookRotation(dir);
            }
        }
    }
}
