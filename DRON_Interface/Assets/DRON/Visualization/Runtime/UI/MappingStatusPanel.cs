using System;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

namespace DRON.Visualization.UI
{
    public class MappingStatusPanel : MonoBehaviour
    {
        [Header("Status UI")]
        [SerializeField] private TextMeshProUGUI trackingStateText;
        [SerializeField] private TextMeshProUGUI pointCountText;
        [SerializeField] private TextMeshProUGUI fpsText;
        [SerializeField] private TextMeshProUGUI latencyText;
        [SerializeField] private Image statusIndicator;

        public void UpdateStatus(bool isTrackingValid, int totalPoints, float fps, float latencyMs, string statusMessage)
        {
            if (trackingStateText != null)
                trackingStateText.text = statusMessage;
            if (pointCountText != null)
                pointCountText.text = $"Points: {totalPoints:N0}";
            if (fpsText != null)
                fpsText.text = $"FPS: {fps:F0}";
            if (latencyText != null)
                latencyText.text = $"Latency: {latencyMs:F0}ms";

            if (statusIndicator != null)
            {
                statusIndicator.color = isTrackingValid ? Color.green : Color.red;
            }
        }
    }
}
