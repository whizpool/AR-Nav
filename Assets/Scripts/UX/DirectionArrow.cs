namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;

    /// <summary>
    /// Rotates the directional arrow child to point toward the next waypoint.
    /// Attach this to the POI prefab. Set arrowTransform to the child
    /// arrow mesh/sprite within the prefab.
    /// </summary>
    public class DirectionArrow : MonoBehaviour
    {
        #region Serialized Fields
        [SerializeField] private Transform arrowTransform;
        #endregion

        #region Public API
        /// <summary>
        /// Call after spawning, passing the outgoing direction of the next leg
        /// (in world space, flattened to XZ).
        /// </summary>
        public void SetDirection(Vector3 outgoingWorldDir)
        {
            if (arrowTransform == null) return;
            outgoingWorldDir.y = 0;
            if (outgoingWorldDir.sqrMagnitude < 0.001f) return;
            arrowTransform.rotation = Quaternion.LookRotation(outgoingWorldDir, Vector3.up);
        }
        #endregion
    }
}
