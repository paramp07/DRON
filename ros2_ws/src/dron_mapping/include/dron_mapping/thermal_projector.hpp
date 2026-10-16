#pragma once

#include "calibration.hpp"
#include "visibility_buffer.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace dron {
namespace mapping {

struct ProjectedThermalPoint {
    cv::Point3f position_left; // Coordinates in Left optical frame (meters)
    float temperature_c;       // Sampled temperature (Celsius)
    float confidence;          // Normalized confidence [0, 1]
    uint8_t valid;             // 1 if valid, 0 if invalid/occluded
    float observation_dt_s;    // Timestamp difference between stereo and thermal
};

class ThermalProjector {
public:
    ThermalProjector(const CameraIntrinsics& thermal_calib,
                     const RigExtrinsics& rig_extrinsics,
                     float depth_tolerance_m = 0.15f);

    /**
     * @brief Projects 3D points from Left Camera frame into the thermal array.
     * @param points_rectified_left Vector of 3D points in rectified left coordinates.
     * @param thermal_matrix 2D floating-point matrix of temperatures in Celsius (CV_32F).
     * @param R1 Rectification matrix (to rotate rectified points back to optical frame).
     * @param dt_s Measured time difference between stereo and thermal acquisition.
     * @return Vector of ProjectedThermalPoint with metric positions and sampled temperatures.
     */
    std::vector<ProjectedThermalPoint> project(
        const std::vector<cv::Point3f>& points_rectified_left,
        const cv::Mat& thermal_matrix,
        const cv::Mat& R1,
        float dt_s);

private:
    CameraIntrinsics thermal_calib_;
    RigExtrinsics rig_extrinsics_;
    VisibilityBuffer visibility_buffer_;
};

} // namespace mapping
} // namespace dron
