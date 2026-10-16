using System;
using System.Collections.Generic;
using UnityEngine;
using DRON.Visualization.Data;

namespace DRON.Visualization.Rendering
{
    /// <summary>
    /// High-performance point cloud renderer utilizing a single managed GraphicsBuffer.
    /// Resizes capacity exponentially to avoid constant reallocation; releases all buffers cleanly on disable.
    /// </summary>
    public class ThermalCloudRenderer : MonoBehaviour
    {
        [Header("Material & Shader")]
        [SerializeField] private Material pointCloudMaterial;
        [SerializeField] private float pointSize = 0.03f;

        private GraphicsBuffer _pointBuffer;
        private int _allocatedCapacity = 0;
        private int _activePointCount = 0;

        public int ActivePointCount => _activePointCount;

        private void OnEnable()
        {
            EnsureBufferCapacity(10000);
        }

        private void OnDisable()
        {
            ReleaseBuffer();
        }

        private void OnDestroy()
        {
            ReleaseBuffer();
        }

        public void UploadPoints(ThermalPoint[] points, int count)
        {
            if (points == null || count <= 0)
            {
                _activePointCount = 0;
                return;
            }

            EnsureBufferCapacity(count);
            _pointBuffer.SetData(points, 0, 0, count);
            _activePointCount = count;
        }

        private void EnsureBufferCapacity(int requiredCount)
        {
            if (_pointBuffer != null && _allocatedCapacity >= requiredCount)
            {
                return;
            }

            ReleaseBuffer();

            // Exponential allocation growth to prevent thrashing
            _allocatedCapacity = Mathf.Max(requiredCount * 2, 65536);
            _pointBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, _allocatedCapacity, ThermalPoint.SizeInBytes);

            if (pointCloudMaterial != null)
            {
                pointCloudMaterial.SetBuffer("_PointsBuffer", _pointBuffer);
            }
        }

        private void ReleaseBuffer()
        {
            if (_pointBuffer != null)
            {
                _pointBuffer.Release();
                _pointBuffer = null;
            }
            _allocatedCapacity = 0;
            _activePointCount = 0;
        }

        private void OnRenderObject()
        {
            if (_pointBuffer == null || _activePointCount <= 0 || pointCloudMaterial == null)
            {
                return;
            }

            pointCloudMaterial.SetBuffer("_PointsBuffer", _pointBuffer);
            pointCloudMaterial.SetFloat("_PointSize", pointSize);
            pointCloudMaterial.SetPass(0);

            // Draw all points in a single draw call with zero CPU instancing
            Graphics.DrawProceduralNow(MeshTopology.Points, _activePointCount);
        }
    }
}
