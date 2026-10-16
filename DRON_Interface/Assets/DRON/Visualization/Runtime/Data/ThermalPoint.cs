using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace DRON.Visualization.Data
{
    [StructLayout(LayoutKind.Sequential)]
    public struct ThermalPoint
    {
        public Vector3 position;       // 12 bytes: Local or World position in Unity coords
        public float temperatureCelsius; // 4 bytes: Measured apparent temperature
        public float confidence;        // 4 bytes: [0.0, 1.0] confidence score
        public uint valid;             // 4 bytes: 1 if valid, 0 if occluded/invalid
        public Color32 displayColor;   // 4 bytes: RGBA mapped from thermal palette
        public float observationDt;    // 4 bytes: Delta-time between stereo and thermal frames

        public static int SizeInBytes => Marshal.SizeOf(typeof(ThermalPoint));
    }
}
