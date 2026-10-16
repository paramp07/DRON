#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

namespace dron {
namespace mapping {

/**
 * @brief Z-buffer on the thermal sensor plane to determine surface visibility
 * and prevent foreground thermal readings from bleeding onto occluded background points.
 */
class VisibilityBuffer {
public:
    VisibilityBuffer(int width, int height, float depth_tolerance_m = 0.15f);
    void reset();

    /**
     * @brief Rasterizes point into the z-buffer, recording closest depth.
     */
    void insert_depth(int u, int v, float z_thermal);

    /**
     * @brief Tests if a point at (u, v, z) is visible (i.e. close to closest recorded depth).
     */
    bool is_visible(int u, int v, float z_thermal) const;

private:
    int width_;
    int height_;
    float depth_tolerance_m_;
    std::vector<float> depth_buffer_;
};

} // namespace mapping
} // namespace dron
