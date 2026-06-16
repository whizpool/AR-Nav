namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Data container for a single Point of Interest.
    /// Configure entirely from the Inspector — no code changes needed to add new POIs.
    /// Same namespace as NavigationController and GPSCoordinate.
    /// </summary>
    [System.Serializable]
    public class PointOfInterest
    {
        [Header("Location")]
        [Tooltip("Latitude from Google My Maps or any GPS source")]
        public double latitude;

        [Tooltip("Longitude from Google My Maps or any GPS source")]
        public double longitude;

        [Tooltip("Leave 0 — PoiManager will use terrain anchor altitude automatically")]
        public double altitudeOverride = 0;

        [Header("Content")]
        [Tooltip("Short display name shown in the AR popup panel")]
        public string title = "Point of Interest";

        [Tooltip("Longer description shown when user enters the radius")]
        [TextArea(2, 5)]
        public string description = "Description here.";

        [Header("Trigger")]
        [Tooltip("User must be within this distance (meters) to see the popup")]
        [Range(5f, 200f)]
        public float triggerRadiusMeters = 30f;

        [Tooltip("How long (seconds) to wait before showing again after dismiss. 0 = never re-show.")]
        [Range(0f, 300f)]
        public float reshowCooldownSeconds = 0f;

        [Header("3D Anchor Visual")]
        [Tooltip("3D prefab placed at this POI's geospatial anchor. Leave null to use the default PoiManager.defaultPoiPrefab.")]
        public GameObject poiPrefab;

        [Tooltip("Height offset above terrain in meters (0 = ground level)")]
        [Range(0f, 10f)]
        public float heightAboveTerrain = 0f;

        [Tooltip("Additional Y rotation applied to the placed 3D object")]
        [Range(0f, 360f)]
        public float yRotationOverride = 0f;

        [HideInInspector]
        public float lastDismissedTime = -999f;

        public bool IsOnCooldown()
        {
            if (reshowCooldownSeconds <= 0f) return lastDismissedTime > -999f;
            return Time.time - lastDismissedTime < reshowCooldownSeconds;
        }

        public override string ToString() =>
            $"POI '{title}' ({latitude:F6}, {longitude:F6}, r={triggerRadiusMeters}m)";
    }
}
