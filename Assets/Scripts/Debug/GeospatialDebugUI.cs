using UnityEngine;
using TMPro;
using Google.XR.ARCoreExtensions.Samples.Geospatial;

public class GeospatialDebugUI : MonoBehaviour
{
    public TextMeshProUGUI debugText;
    [Tooltip("How many seconds between UI updates (e.g. 0.2s = 5 updates/sec)")]
    public float updateInterval = 0.2f;

    private NavigationController navControl;
    private string lastStatus = "Initializing...";
    private readonly System.Text.StringBuilder sb = new System.Text.StringBuilder();
    
    private float nextUpdateTime;
    private float fpsAccumulator = 0f;
    private int fpsSamples = 0;

    void Start()
    {
        navControl = Object.FindFirstObjectByType<NavigationController>();
        if (navControl != null)
        {
            navControl.OnStatusUpdate += HandleStatusUpdate;
        }
        
        if (debugText == null)
            debugText = GetComponent<TextMeshProUGUI>();
    }

    void OnDestroy()
    {
        if (navControl != null)
        {
            navControl.OnStatusUpdate -= HandleStatusUpdate;
        }
    }

    private void HandleStatusUpdate(string status)
    {
        lastStatus = status;
    }

    void Update()
    {
        // Accumulate FPS stats every frame
        float dt = Time.unscaledDeltaTime;
        if (dt > 0f)
        {
            fpsAccumulator += 1f / dt;
            fpsSamples++;
        }

        if (Time.time < nextUpdateTime)
            return;

        nextUpdateTime = Time.time + updateInterval;

        if (navControl == null) 
        {
            navControl = Object.FindFirstObjectByType<NavigationController>();
            if (navControl != null)
            {
                navControl.OnStatusUpdate += HandleStatusUpdate;
            }
            return;
        }

        float averageFps = fpsSamples > 0 ? fpsAccumulator / fpsSamples : 0f;
        fpsAccumulator = 0f;
        fpsSamples = 0;

        sb.Clear();
        sb.AppendLine("<b><color=#00FF00>DEBUG INFO</color></b>");
        sb.Append("FPS: ").Append(averageFps.ToString("F1")).AppendLine();
        
        if (navControl.earthManager != null && navControl.earthManager.EarthState == Google.XR.ARCoreExtensions.EarthState.Enabled)
        {
            var pose = navControl.earthManager.CameraGeospatialPose;
            var tracking = navControl.earthManager.EarthTrackingState;
            
            sb.Append("Tracking: <color=");
            sb.Append(tracking == UnityEngine.XR.ARSubsystems.TrackingState.Tracking ? "#00FF00" : "#FF0000");
            sb.Append(">").Append(tracking.ToString()).AppendLine("</color>");
            
            if (tracking == UnityEngine.XR.ARSubsystems.TrackingState.Tracking)
            {
                sb.Append("Lat: ").Append(pose.Latitude.ToString("F7")).AppendLine();
                sb.Append("Lng: ").Append(pose.Longitude.ToString("F7")).AppendLine();
                sb.Append("Alt: ").Append(pose.Altitude.ToString("F2")).AppendLine("m");
                sb.Append("H-Acc: ").Append(pose.HorizontalAccuracy.ToString("F2")).AppendLine("m");
                sb.Append("V-Acc: ").Append(pose.VerticalAccuracy.ToString("F2")).AppendLine("m");
            }
        }
        else
        {
            sb.AppendLine("Earth: Disabled");
        }
        
        sb.AppendLine();
        sb.Append("Status: <i>").Append(lastStatus).AppendLine("</i>");
        
        if (debugText != null)
        {
            // SetText(StringBuilder) avoids allocating memory for a string representation
            debugText.SetText(sb);
        }
    }
}
