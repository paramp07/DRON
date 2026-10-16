using System;
using UnityEngine;
using DRON.Visualization.Data;

namespace DRON.Visualization.Transport
{
    /// <summary>
    /// Legacy Ros2ForUnity adapter implementing IRobotMapTransport.
    /// Kept for backwards compatibility testing on supported Linux builds.
    /// </summary>
    public class NativeRos2MapTransport : MonoBehaviour, IRobotMapTransport
    {
        public event Action<MapChunk> OnChunkReceived;
        public event Action<ulong[], Vector3[], Quaternion[], ulong> OnPoseGraphUpdated;
        public event Action<string> OnStatusReceived;

        public bool IsConnected { get; private set; } = false;

        public void Connect()
        {
            IsConnected = true;
            Debug.Log("[NativeRos2MapTransport] Initializing native DDS domain...");
        }

        public void Disconnect()
        {
            IsConnected = false;
        }

        public void RequestMapSnapshot(string sessionId)
        {
            Debug.Log($"[NativeRos2MapTransport] Snapshot service request for {sessionId}");
        }
    }
}
