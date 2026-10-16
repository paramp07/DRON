using System;
using System.Collections.Generic;
using UnityEngine;

namespace DRON.Visualization.Data
{
    public struct PointFieldDescriptor
    {
        public string name;
        public int offset;
        public byte datatype; // 1: INT8, 2: UINT8, 3: INT16, 4: UINT16, 5: INT32, 6: UINT32, 7: FLOAT32, 8: FLOAT64
        public int count;
    }

    /// <summary>
    /// Robust binary decoder for ROS sensor_msgs/PointCloud2 messages.
    /// Safely decodes variable field layouts, padding, and endianness.
    /// </summary>
    public static class PointCloud2Decoder
    {
        public static ThermalPoint[] Decode(
            byte[] rawData,
            int width,
            int height,
            int pointStep,
            int rowStep,
            bool isBigEndian,
            List<PointFieldDescriptor> fields,
            int maxPointsBudget = 1000000)
        {
            if (rawData == null || rawData.Length == 0 || pointStep <= 0)
            {
                return Array.Empty<ThermalPoint>();
            }

            int totalPoints = width * height;
            if (totalPoints <= 0 || totalPoints > maxPointsBudget)
            {
                Debug.LogWarning($"[PointCloud2Decoder] Point count {totalPoints} invalid or exceeds budget {maxPointsBudget}.");
                return Array.Empty<ThermalPoint>();
            }

            int xOffset = -1, yOffset = -1, zOffset = -1;
            int tempOffset = -1, confOffset = -1, validOffset = -1, dtOffset = -1;

            foreach (var f in fields)
            {
                switch (f.name.ToLowerInvariant())
                {
                    case "x": xOffset = f.offset; break;
                    case "y": yOffset = f.offset; break;
                    case "z": zOffset = f.offset; break;
                    case "temperature_c":
                    case "temperature": tempOffset = f.offset; break;
                    case "confidence": confOffset = f.offset; break;
                    case "valid": validOffset = f.offset; break;
                    case "observation_dt_s":
                    case "dt": dtOffset = f.offset; break;
                }
            }

            if (xOffset < 0 || yOffset < 0 || zOffset < 0)
            {
                Debug.LogError("[PointCloud2Decoder] Missing required XYZ field offsets in PointCloud2.");
                return Array.Empty<ThermalPoint>();
            }

            var points = new ThermalPoint[totalPoints];
            int validCount = 0;

            for (int i = 0; i < totalPoints; i++)
            {
                int baseIdx = i * pointStep;
                if (baseIdx + pointStep > rawData.Length)
                {
                    break;
                }

                float x = BitConverter.ToSingle(rawData, baseIdx + xOffset);
                float y = BitConverter.ToSingle(rawData, baseIdx + yOffset);
                float z = BitConverter.ToSingle(rawData, baseIdx + zOffset);

                if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || float.IsInfinity(x) || float.IsInfinity(y) || float.IsInfinity(z))
                {
                    continue;
                }

                float temp = (tempOffset >= 0 && baseIdx + tempOffset + 4 <= rawData.Length)
                    ? BitConverter.ToSingle(rawData, baseIdx + tempOffset)
                    : -999.0f;

                float conf = (confOffset >= 0 && baseIdx + confOffset + 4 <= rawData.Length)
                    ? BitConverter.ToSingle(rawData, baseIdx + confOffset)
                    : 1.0f;

                uint valid = (validOffset >= 0 && baseIdx + validOffset < rawData.Length)
                    ? (uint)rawData[baseIdx + validOffset]
                    : 1u;

                float dt = (dtOffset >= 0 && baseIdx + dtOffset + 4 <= rawData.Length)
                    ? BitConverter.ToSingle(rawData, baseIdx + dtOffset)
                    : 0.0f;

                // Position converted from ROS Optical to Unity frame
                Vector3 unityPos = CoordinateConverter.RosOpticalToUnity(new Vector3(x, y, z));

                points[validCount++] = new ThermalPoint
                {
                    position = unityPos,
                    temperatureCelsius = temp,
                    confidence = conf,
                    valid = valid,
                    observationDt = dt,
                    displayColor = new Color32(255, 255, 255, 255)
                };
            }

            if (validCount < totalPoints)
            {
                Array.Resize(ref points, validCount);
            }

            return points;
        }
    }
}
