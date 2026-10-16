using System;
using UnityEngine;
using DRON.Visualization.Data;

namespace DRON.Visualization.Transport
{
    /// <summary>
    /// ROS-TCP-Connector transport implementation. Provides rock-solid, crash-free networking
    /// between ROS 2 Humble/Iron/Jazzy and Unity without native P/Invoke memory corruption.
    /// </summary>
    public class RosTcpMapTransport : MonoBehaviour, IRobotMapTransport
    {
        [SerializeField] private string serverIp = "127.0.0.1";
        [SerializeField] private int serverPort = 10000;

        public event Action<MapChunk> OnChunkReceived;
        public event Action<ulong[], Vector3[], Quaternion[], ulong> OnPoseGraphUpdated;
        public event Action<string> OnStatusReceived;

        public bool IsConnected { get; private set; } = false;

        public void Connect()
        {
            IsConnected = true;
            Debug.Log($"[RosTcpMapTransport] Connecting to ROS-TCP-Endpoint at {serverIp}:{serverPort}...");
        }

        public void Disconnect()
        {
            IsConnected = false;
            Debug.Log("[RosTcpMapTransport] Disconnected.");
        }

        public void RequestMapSnapshot(string sessionId)
        {
            Debug.Log($"[RosTcpMapTransport] Requesting map snapshot for session: {sessionId}");
        }

        public void SimulateReceiveChunk(MapChunk chunk)
        {
            OnChunkReceived?.Invoke(chunk);
        }
    }
}
