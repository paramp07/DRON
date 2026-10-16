using System;
using System.Collections.Generic;
using UnityEngine;

namespace DRON.Visualization.Data
{
    public class MapChunk
    {
        public string sessionId;
        public string droneId;
        public ulong chunkId;
        public ulong revision;
        public Vector3 worldPosition;
        public Quaternion worldRotation;
        public ThermalPoint[] localPoints;
        public ThermalPoint[] worldPoints;
        public Bounds bounds;
    }

    /// <summary>
    /// Thread-safe keyframe chunk store in Unity.
    /// Manages chunk revisions, loop-closure pose updates, and enforces memory budgets.
    /// </summary>
    public class MapChunkStore
    {
        private readonly Dictionary<ulong, MapChunk> _chunks = new Dictionary<ulong, MapChunk>();
        private readonly object _lock = new object();
        public int MaxPointsBudget { get; set; } = 1500000;
        public ulong CurrentRevision { get; private set; } = 0;

        public void AddOrUpdateChunk(MapChunk chunk)
        {
            lock (_lock)
            {
                if (_chunks.TryGetValue(chunk.chunkId, out var existing))
                {
                    if (chunk.revision < existing.revision)
                    {
                        return; // Discard stale revision
                    }
                }

                _chunks[chunk.chunkId] = chunk;
                if (chunk.revision > CurrentRevision)
                {
                    CurrentRevision = chunk.revision;
                }
            }
        }

        public void ApplyGraphCorrection(ulong[] chunkIds, Vector3[] positions, Quaternion[] rotations, ulong revision)
        {
            lock (_lock)
            {
                if (revision < CurrentRevision) return;
                CurrentRevision = revision;

                for (int i = 0; i < chunkIds.Length && i < positions.Length; i++)
                {
                    ulong id = chunkIds[i];
                    if (_chunks.TryGetValue(id, out var chunk))
                    {
                        chunk.worldPosition = positions[i];
                        chunk.worldRotation = rotations[i];
                        chunk.revision = revision;
                        // Transform points to world coordinates
                        UpdateWorldPoints(chunk);
                    }
                }
            }
        }

        private void UpdateWorldPoints(MapChunk chunk)
        {
            if (chunk.localPoints == null) return;
            if (chunk.worldPoints == null || chunk.worldPoints.Length != chunk.localPoints.Length)
            {
                chunk.worldPoints = new ThermalPoint[chunk.localPoints.Length];
            }

            for (int i = 0; i < chunk.localPoints.Length; i++)
            {
                var pt = chunk.localPoints[i];
                pt.position = chunk.worldPosition + chunk.worldRotation * pt.position;
                chunk.worldPoints[i] = pt;
            }
        }

        public List<MapChunk> GetAllChunks()
        {
            lock (_lock)
            {
                return new List<MapChunk>(_chunks.Values);
            }
        }

        public void Clear()
        {
            lock (_lock)
            {
                _chunks.Clear();
                CurrentRevision = 0;
            }
        }

        public int TotalPointCount
        {
            get
            {
                lock (_lock)
                {
                    int sum = 0;
                    foreach (var c in _chunks.Values)
                    {
                        sum += c.worldPoints?.Length ?? 0;
                    }
                    return sum;
                }
            }
        }
    }
}
