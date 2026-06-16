using UnityEngine;
using TMPro;
using Google.XR.ARCoreExtensions.Samples.Geospatial;

public class GeospatialDebugUI : MonoBehaviour
{
    public TextMeshProUGUI debugText;
    private NavigationController navControl;
    private string lastStatus = "Initializing...";

    void Start()
    {
        navControl = Object.FindFirstObjectByType<NavigationController>();
        if (navControl != null)
        {
            navControl.OnStatusUpdate += (s) => lastStatus = s;
        }
        
        if (debugText == null)
            debugText = GetComponent<TextMeshProUGUI>();
    }

    void Update()
    {
        if (navControl == null) 
        {
            navControl = Object.FindFirstObjectByType<NavigationController>();
            return;
        }
        
        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine($"<b><color=#00FF00>DEBUG INFO</color></b>");
        sb.AppendLine($"FPS: {(1f / Time.unscaledDeltaTime):F1}");
        
        if (navControl.earthManager != null && navControl.earthManager.EarthState == Google.XR.ARCoreExtensions.EarthState.Enabled)
        {
            var pose = navControl.earthManager.CameraGeospatialPose;
            var tracking = navControl.earthManager.EarthTrackingState;
            
            sb.AppendLine($"Tracking: <color={(tracking == UnityEngine.XR.ARSubsystems.TrackingState.Tracking ? "#00FF00" : "#FF0000")}>{tracking}</color>");
            
            if (tracking == UnityEngine.XR.ARSubsystems.TrackingState.Tracking)
            {
                sb.AppendLine($"Lat: {pose.Latitude:F7}");
                sb.AppendLine($"Lng: {pose.Longitude:F7}");
                sb.AppendLine($"Alt: {pose.Altitude:F2}m");
                sb.AppendLine($"H-Acc: {pose.HorizontalAccuracy:F2}m");
                sb.AppendLine($"V-Acc: {pose.VerticalAccuracy:F2}m");
            }
        }
        else
        {
            sb.AppendLine("Earth: Disabled");
        }
        
        sb.AppendLine($"\nStatus: <i>{lastStatus}</i>");
        
        if (debugText != null)
            debugText.text = sb.ToString();
    }
}
