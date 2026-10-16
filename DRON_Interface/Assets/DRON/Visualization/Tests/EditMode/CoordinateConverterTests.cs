using NUnit.Framework;
using UnityEngine;
using DRON.Visualization.Data;

namespace DRON.Visualization.Tests
{
    public class CoordinateConverterTests
    {
        [Test]
        public void RosFluToUnity_ConvertsBasisCorrectly()
        {
            // ROS FLU: X forward (1), Y left (2), Z up (3)
            Vector3 rosPos = new Vector3(1.0f, 2.0f, 3.0f);
            
            // Expected Unity: X right (-Y = -2), Y up (Z = 3), Z forward (X = 1)
            Vector3 unityPos = CoordinateConverter.RosFluToUnity(rosPos);

            Assert.AreEqual(-2.0f, unityPos.x, 1e-4f);
            Assert.AreEqual(3.0f, unityPos.y, 1e-4f);
            Assert.AreEqual(1.0f, unityPos.z, 1e-4f);

            // Invert back
            Vector3 roundTrip = CoordinateConverter.UnityToRosFlu(unityPos);
            Assert.AreEqual(rosPos.x, roundTrip.x, 1e-4f);
            Assert.AreEqual(rosPos.y, roundTrip.y, 1e-4f);
            Assert.AreEqual(rosPos.z, roundTrip.z, 1e-4f);
        }

        [Test]
        public void RosOpticalToUnity_InvertsVerticalAxis()
        {
            // Optical: X right (4), Y down (5), Z forward (6)
            Vector3 optical = new Vector3(4.0f, 5.0f, 6.0f);
            Vector3 unity = CoordinateConverter.RosOpticalToUnity(optical);

            Assert.AreEqual(4.0f, unity.x, 1e-4f);
            Assert.AreEqual(-5.0f, unity.y, 1e-4f);
            Assert.AreEqual(6.0f, unity.z, 1e-4f);
        }
    }
}
