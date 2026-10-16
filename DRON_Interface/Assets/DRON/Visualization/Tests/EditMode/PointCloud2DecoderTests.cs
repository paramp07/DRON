using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using DRON.Visualization.Data;

namespace DRON.Visualization.Tests
{
    public class PointCloud2DecoderTests
    {
        [Test]
        public void Decode_ExtractsCoordinatesAndTemperatures()
        {
            // Create synthetic byte buffer for 2 points with XYZ + Temperature + Confidence
            // point_step = 20 bytes: x(4), y(4), z(4), temp(4), conf(4)
            int pointStep = 20;
            byte[] buffer = new byte[pointStep * 2];

            // Point 0: (1.0, 2.0, 3.0), temp = 45.5 C, conf = 0.95
            Buffer.BlockCopy(BitConverter.GetBytes(1.0f), 0, buffer, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(2.0f), 0, buffer, 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(3.0f), 0, buffer, 8, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(45.5f), 0, buffer, 12, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(0.95f), 0, buffer, 16, 4);

            // Point 1: (-0.5, 0.0, 2.5), temp = 88.0 C, conf = 0.99
            Buffer.BlockCopy(BitConverter.GetBytes(-0.5f), 0, buffer, 20, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(0.0f), 0, buffer, 24, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(2.5f), 0, buffer, 28, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(88.0f), 0, buffer, 32, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(0.99f), 0, buffer, 36, 4);

            var fields = new List<PointFieldDescriptor>
            {
                new PointFieldDescriptor { name = "x", offset = 0, datatype = 7, count = 1 },
                new PointFieldDescriptor { name = "y", offset = 4, datatype = 7, count = 1 },
                new PointFieldDescriptor { name = "z", offset = 8, datatype = 7, count = 1 },
                new PointFieldDescriptor { name = "temperature_c", offset = 12, datatype = 7, count = 1 },
                new PointFieldDescriptor { name = "confidence", offset = 16, datatype = 7, count = 1 }
            };

            var points = PointCloud2Decoder.Decode(buffer, 2, 1, pointStep, pointStep * 2, false, fields);

            Assert.AreEqual(2, points.Length);
            Assert.AreEqual(1.0f, points[0].position.x, 1e-4f);
            Assert.AreEqual(-2.0f, points[0].position.y, 1e-4f); // Optical to Unity Y flip
            Assert.AreEqual(3.0f, points[0].position.z, 1e-4f);
            Assert.AreEqual(45.5f, points[0].temperatureCelsius, 1e-4f);
            Assert.AreEqual(0.95f, points[0].confidence, 1e-4f);

            Assert.AreEqual(-0.5f, points[1].position.x, 1e-4f);
            Assert.AreEqual(0.0f, points[1].position.y, 1e-4f);
            Assert.AreEqual(2.5f, points[1].position.z, 1e-4f);
            Assert.AreEqual(88.0f, points[1].temperatureCelsius, 1e-4f);
        }
    }
}
