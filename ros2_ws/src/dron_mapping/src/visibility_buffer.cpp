#include "dron_mapping/visibility_buffer.hpp"
#include <limits>

namespace dron {
namespace mapping {

VisibilityBuffer::VisibilityBuffer(int width, int height, float depth_tolerance_m)
    : width_(width), height_(height), depth_tolerance_m_(depth_tolerance_m) {
    reset();
}

void VisibilityBuffer::reset() {
    depth_buffer_.assign(width_ * height_, std::numeric_limits<float>::infinity());
}

void VisibilityBuffer::insert_depth(int u, int v, float z_thermal) {
    if (u < 0 || u >= width_ || v < 0 || v >= height_ || z_thermal <= 0.0f) {
        return;
    }
    int idx = v * width_ + u;
    if (z_thermal < depth_buffer_[idx]) {
        depth_buffer_[idx] = z_thermal;
    }
}

bool VisibilityBuffer::is_visible(int u, int v, float z_thermal) const {
    if (u < 0 || u >= width_ || v < 0 || v >= height_ || z_thermal <= 0.0f) {
        return false;
    }
    int idx = v * width_ + u;
    float closest_z = depth_buffer_[idx];
    if (std::isinf(closest_z)) {
        return false;
    }
    // If point depth is within depth tolerance of closest surface, it is visible
    return (z_thermal <= closest_z + depth_tolerance_m_);
}

} // namespace mapping
} // namespace dron
