using UnityEngine;

public class BillboardCanvas : MonoBehaviour
{
    void Update()
    {
        if (Camera.main == null) return;
        transform.LookAt(Camera.main.transform);
        transform.Rotate(0, 180f, 0);
    }
}