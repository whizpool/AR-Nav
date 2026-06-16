using UnityEngine;
using UnityEngine.XR.ARFoundation;

public class ARStartupDebug : MonoBehaviour
{
    private ARSession _arSession;

    void Awake()
    {
        Debug.Log("=== ARStartupDebug: Awake ===");
        _arSession = FindObjectOfType<ARSession>();
    }

    void Start()
    {
        Debug.Log("=== ARStartupDebug: Start ===");
        Debug.Log($"ARSession supported: {ARSession.state}");
        Debug.Log($"Camera enabled: {Camera.main?.enabled}");
        Debug.Log($"ARSession GameObject active: {_arSession?.gameObject.activeSelf}");
        StartCoroutine(TrackARState());
    }

    System.Collections.IEnumerator TrackARState()
    {
        int tick = 0;
        while (tick < 10)
        {
            yield return new WaitForSeconds(1f);
            tick++;
            Debug.Log($"=== Tick {tick}: ARState={ARSession.state} " +
                      $"Frame={Time.frameCount} " +
                      $"Camera={Camera.main?.enabled}");
        }
    }
}