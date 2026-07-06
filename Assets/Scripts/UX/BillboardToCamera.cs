namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Attach to the turn indicator prefab's root.
    /// Performs Y-axis billboarding (stays upright, faces camera horizontally).
    /// </summary>
    public class BillboardToCamera : MonoBehaviour
    {
        #region Private Fields
        private Transform _cam;
        #endregion

        #region Unity Lifecycle
        void Start() => _cam = Camera.main.transform;

        void LateUpdate()
        {
            if (_cam == null) return;

            // Only rotate around Y axis (keeps POI upright)
            Vector3 lookDir = _cam.position - transform.position;
            lookDir.y = 0; // flatten
            if (lookDir.sqrMagnitude > 0.001f)
            {
                transform.rotation = Quaternion.LookRotation(-lookDir, Vector3.up);
            }
        }
        #endregion
    }
}
