#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace dron {
namespace mapping {

struct CameraIntrinsics {
    std::string camera_name;
    int width = 0;
    int height = 0;
    cv::Mat K; // 3x3 double
    cv::Mat D; // 1x5 double
    cv::Mat R; // 3x3 rectification matrix
    cv::Mat P; // 3x4 projection matrix
};

struct RigExtrinsics {
    std::string rig_id;
    std::string calibration_id;
    cv::Mat R_stereo;        // 3x3 Right from Left
    cv::Mat T_stereo;        // 3x1 Right from Left
    cv::Mat R_thermal_left;  // 3x3 Thermal from Left
    cv::Mat T_thermal_left;  // 3x1 Thermal from Left
    cv::Mat Q;               // 4x4 Disparity-to-depth reprojection matrix
};

} // namespace mapping
} // namespace dron
