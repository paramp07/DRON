using System;
using UnityEngine;
using TMPro;

namespace DRON.Visualization.UI
{
    public class MapInspector : MonoBehaviour
    {
        [Header("UI Text References")]
        [SerializeField] private TextMeshProUGUI temperatureText;
        [SerializeField] private TextMeshProUGUI confidenceText;
        [SerializeField] private TextMeshProUGUI positionText;
        [SerializeField] private TextMeshProUGUI timestampText;

        public void InspectPoint(Vector3 worldPos, float tempCelsius, float confidence, float ageSeconds)
        {
            if (temperatureText != null)
                temperatureText.text = $"Temp: {tempCelsius:F1} °C";
            if (confidenceText != null)
                confidenceText.text = $"Conf: {confidence * 100:F0}%";
            if (positionText != null)
                positionText.text = $"Pos: ({worldPos.x:F2}, {worldPos.y:F2}, {worldPos.z:F2})m";
            if (timestampText != null)
                timestampText.text = $"Age: {ageSeconds:F1}s";
        }
    }
}
