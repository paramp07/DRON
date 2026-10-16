using UnityEngine;

namespace DRON.Visualization.Data
{
    /// <summary>
    /// Strict, tested coordinate transformation boundary between ROS (FLU / Optical) and Unity (Left-Handed Y-up).
    /// </summary>
    public static class CoordinateConverter
    {
        /// <summary>
        /// Converts ROS FLU (Forward-Left-Up) world positions to Unity (X-Right, Y-Up, Z-Forward).
        /// Standard transformation: Unity.X = -ROS.Y, Unity.Y = ROS.Z, Unity.Z = ROS.X
        /// </summary>
        public static Vector3 RosFluToUnity(Vector3 rosPosition)
        {
            return new Vector3(-rosPosition.y, rosPosition.z, rosPosition.x);
        }

        /// <summary>
        /// Converts Unity world positions back to ROS FLU.
        /// </summary>
        public static Vector3 UnityToRosFlu(Vector3 unityPosition)
        {
            return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
        }

        /// <summary>
        /// Converts ROS camera optical coordinates (X-Right, Y-Down, Z-Forward) to Unity frame.
        /// </summary>
        public static Vector3 RosOpticalToUnity(Vector3 opticalPosition)
        {
            return new Vector3(opticalPosition.x, -opticalPosition.y, opticalPosition.z);
        }

        /// <summary>
        /// Converts ROS FLU quaternion to Unity quaternion.
        /// </summary>
        public static Quaternion RosFluToUnity(Quaternion rosQuat)
        {
            // Basis transformation: q_unity = (-q_ros.y, q_ros.z, q_ros.x, q_ros.w)
            return new Quaternion(-rosQuat.y, rosQuat.z, rosQuat.x, rosQuat.w);
        }
    }
}
