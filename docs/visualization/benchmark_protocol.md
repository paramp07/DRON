# Benchmark & Acceptance Protocol

## Acceptance Criteria
1. **Stereo Calibration**: Vertical epipolar residual median < 0.5 px, p95 < 1.0 px.
2. **Metric Depth Accuracy**: Median absolute error < 5% on planar targets at 1m to 6m range.
3. **Thermal Alignment**: Feature reprojection error < 1.0 native thermal pixel (approx 0.05m at 5m distance).
4. **Acquisition Skew**: $\Delta t < 50$ ms between paired stereo and thermal frames under normal drone velocities (< 1.0 m/s).
5. **Real-time Performance**:
   - Ground station processing latency < 150 ms per frame.
   - Unity rendering frame rate $\ge$ 30 FPS with up to 1,500,000 active points.
6. **Stability**: Zero memory leaks or crashes over a 20-minute continuous streaming session.
