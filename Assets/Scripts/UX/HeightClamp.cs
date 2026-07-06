namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Clamps the POI's world Y so it stays within a comfortable
    /// viewing band relative to the camera.
    /// </summary>
    public class HeightClamp : MonoBehaviour
    {
        #region Serialized Fields
        [SerializeField] private float minHeightAboveCamera = -1.0f; // allow slightly below eye
        [SerializeField] private float maxHeightAboveCamera =  2.0f; // never too high
        #endregion

        #region Private Fields
        private Transform _cam;
        #endregion

        #region Unity Lifecycle
        void Start() => _cam = Camera.main.transform;

        void LateUpdate()
        {
            if (_cam == null) return;
            Vector3 pos = transform.position;
            float camY = _cam.position.y;
            pos.y = Mathf.Clamp(pos.y, camY + minHeightAboveCamera, camY + maxHeightAboveCamera);
            transform.position = pos;
        }
        #endregion
    }
}
