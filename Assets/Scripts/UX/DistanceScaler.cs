namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Scales the POI based on distance to camera so it maintains a
    /// roughly constant angular size, with min/max clamps.
    /// </summary>
    public class DistanceScaler : MonoBehaviour
    {
        #region Serialized Fields
        [SerializeField] private float referenceDistance = 15f;  // meters
        [SerializeField] private float referenceScale   = 1.0f; // scale at ref distance
        [SerializeField] private float minScale         = 0.4f;
        [SerializeField] private float maxScale         = 3.0f;
        [SerializeField] private float smoothSpeed      = 5f;
        #endregion

        #region Private Fields
        private Transform _cam;
        private Vector3   _baseScale;
        #endregion

        #region Unity Lifecycle
        void Start()
        {
            _cam = Camera.main.transform;
            _baseScale = transform.localScale;
        }

        void LateUpdate()
        {
            if (_cam == null) return;
            float dist = Vector3.Distance(_cam.position, transform.position);
            float factor = (dist / referenceDistance) * referenceScale;
            factor = Mathf.Clamp(factor, minScale, maxScale);

            Vector3 target = _baseScale * factor;
            transform.localScale = Vector3.Lerp(
                transform.localScale, target, smoothSpeed * Time.deltaTime);
        }
        #endregion
    }
}
