using System;
using UnityEngine;
using DRON.Visualization.Data;

namespace DRON.Visualization.Transport
{
    public interface IRobotMapTransport
    {
        event Action<MapChunk> OnChunkReceived;
        event Action<ulong[], Vector3[], Quaternion[], ulong> OnPoseGraphUpdated;
        event Action<string> OnStatusReceived;

        bool IsConnected { get; }
        void Connect();
        void Disconnect();
        void RequestMapSnapshot(string sessionId);
    }
}
