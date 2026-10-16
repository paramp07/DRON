using System;
using UnityEngine;

namespace DRON.Visualization.Rendering
{
    public enum ColormapType
    {
        Turbo,
        Ironbow,
        Inferno
    }

    /// <summary>
    /// Translates numeric Celsius temperatures to false-color RGB palettes.
    /// Recolorization occurs without recalculating 3D geometry.
    /// </summary>
    public class ThermalPalette
    {
        public float MinTempCelsius { get; set; } = 20.0f;
        public float MaxTempCelsius { get; set; } = 100.0f;
        public ColormapType Colormap { get; set; } = ColormapType.Turbo;
        public float StaleThresholdSeconds { get; set; } = 10.0f;

        public Color32 Evaluate(float tempCelsius, float ageSeconds = 0.0f, bool isValid = true)
        {
            if (!isValid || tempCelsius <= -500.0f)
            {
                return new Color32(80, 80, 80, 255); // Gray for unmeasured / invalid
            }

            float t = Mathf.Clamp01((tempCelsius - MinTempCelsius) / Mathf.Max(0.1f, (MaxTempCelsius - MinTempCelsius)));

            Color c;
            switch (Colormap)
            {
                case ColormapType.Ironbow:
                    c = SampleIronbow(t);
                    break;
                case ColormapType.Inferno:
                    c = SampleInferno(t);
                    break;
                case ColormapType.Turbo:
                default:
                    c = SampleTurbo(t);
                    break;
            }

            // Stale fade: if observation is old, desaturate toward dark gray
            if (ageSeconds > StaleThresholdSeconds)
            {
                float staleFactor = Mathf.Clamp01((ageSeconds - StaleThresholdSeconds) / 20.0f);
                c = Color.Lerp(c, new Color(0.3f, 0.3f, 0.3f, 1.0f), staleFactor * 0.7f);
            }

            return (Color32)c;
        }

        private static Color SampleTurbo(float x)
        {
            // Polynomial approximation for Google Turbo colormap
            float r = 0.1357f + x * (4.5974f + x * (-42.3277f + x * (130.5887f + x * (-150.5664f + x * 58.1375f))));
            float g = 0.0914f + x * (2.1856f + x * (4.8052f + x * (-14.0195f + x * (4.2109f + x * 2.7747f))));
            float b = 0.1067f + x * (12.5847f + x * (-67.5714f + x * (131.6257f + x * (-118.0044f + x * 38.6015f))));
            return new Color(Mathf.Clamp01(r), Mathf.Clamp01(g), Mathf.Clamp01(b), 1.0f);
        }

        private static Color SampleIronbow(float x)
        {
            // Classic FLIR Ironbow approximation
            if (x < 0.25f) return Color.Lerp(Color.black, new Color(0.2f, 0.0f, 0.5f), x / 0.25f);
            if (x < 0.50f) return Color.Lerp(new Color(0.2f, 0.0f, 0.5f), new Color(0.8f, 0.1f, 0.4f), (x - 0.25f) / 0.25f);
            if (x < 0.75f) return Color.Lerp(new Color(0.8f, 0.1f, 0.4f), new Color(1.0f, 0.6f, 0.0f), (x - 0.50f) / 0.25f);
            return Color.Lerp(new Color(1.0f, 0.6f, 0.0f), Color.white, (x - 0.75f) / 0.25f);
        }

        private static Color SampleInferno(float x)
        {
            // Perceptually uniform Inferno
            float r = Mathf.Clamp01(x * 1.3f);
            float g = Mathf.Clamp01(Mathf.Pow(x, 2.0f) * 1.1f);
            float b = Mathf.Clamp01(Mathf.Sin(x * Mathf.PI * 0.8f) * 0.5f);
            return new Color(r, g, b, 1.0f);
        }
    }
}
