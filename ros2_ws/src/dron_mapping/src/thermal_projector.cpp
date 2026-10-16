#include "dron_mapping/thermal_projector.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace dron {
namespace mapping {

ThermalProjector::ThermalProjector(const CameraIntrinsics& thermal_calib,
                                   const RigExtrinsics& rig_extrinsics,
                                   float depth_tolerance_m)
    : thermal_calib_(thermal_calib),
      rig_extrinsics_(rig_extrinsics),
      visibility_buffer_(thermal_calib.width, thermal_calib.height, depth_tolerance_m) {
}

std::vector<ProjectedThermalPoint> ThermalProjector::project(
    const std::vector<cv::Point3f>& points_rectified_left,
    const cv::Mat& thermal_matrix,
    const cv::Mat& R1,
    float dt_s) {

    size_t n_points = points_rectified_left.size();
    std::vector<ProjectedThermalPoint> output(n_points);

    if (n_points == 0) {
        return output;
    }

    // Prepare transformation matrices (Double precision)
    cv::Mat R1_inv;
    cv::transpose(R1, R1_inv); // R1 is orthogonal, so transpose is inverse

    cv::Mat R_th = rig_extrinsics_.R_thermal_left;
    cv::Mat T_th = rig_extrinsics_.T_thermal_left;

    std::vector<cv::Point3f> pts_left_optical(n_points);
    std::vector<cv::Point3f> pts_thermal(n_points);
    std::vector<bool> positive_depth(n_points, false);

    // 1. Transform each point: Rectified -> Original Left Optical -> Thermal Frame
    for (size_t i = 0; i < n_points; ++i) {
        const auto& pr = points_rectified_left[i];
        if (std::isnan(pr.x) || std::isnan(pr.y) || std::isnan(pr.z) || pr.z <= 0.05f) {
            continue;
        }

        // P_left = R1^T * P_rectified
        double xl = R1_inv.at<double>(0,0)*pr.x + R1_inv.at<double>(0,1)*pr.y + R1_inv.at<double>(0,2)*pr.z;
        double yl = R1_inv.at<double>(1,0)*pr.x + R1_inv.at<double>(1,1)*pr.y + R1_inv.at<double>(1,2)*pr.z;
        double zl = R1_inv.at<double>(2,0)*pr.x + R1_inv.at<double>(2,1)*pr.y + R1_inv.at<double>(2,2)*pr.z;
        pts_left_optical[i] = cv::Point3f(static_cast<float>(xl), static_cast<float>(yl), static_cast<float>(zl));

        // P_th = R_th * P_left + T_th
        double xt = R_th.at<double>(0,0)*xl + R_th.at<double>(0,1)*yl + R_th.at<double>(0,2)*zl + T_th.at<double>(0,0);
        double yt = R_th.at<double>(1,0)*xl + R_th.at<double>(1,1)*yl + R_th.at<double>(1,2)*zl + T_th.at<double>(1,0);
        double zt = R_th.at<double>(2,0)*xl + R_th.at<double>(2,1)*yl + R_th.at<double>(2,2)*zl + T_th.at<double>(2,0);
        pts_thermal[i] = cv::Point3f(static_cast<float>(xt), static_cast<float>(yt), static_cast<float>(zt));

        if (zt > 0.05f) {
            positive_depth[i] = true;
        }
    }

    // 2. Perspective Projection with lens distortion
    std::vector<cv::Point2f> projected_uv(n_points);
    cv::Mat zero_rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat zero_tvec = cv::Mat::zeros(3, 1, CV_64F);

    cv::projectPoints(pts_thermal, zero_rvec, zero_tvec, thermal_calib_.K, thermal_calib_.D, projected_uv);

    // 3. Populate Visibility Z-buffer
    visibility_buffer_.reset();
    for (size_t i = 0; i < n_points; ++i) {
        if (!positive_depth[i]) continue;
        int u_int = static_cast<int>(std::round(projected_uv[i].x));
        int v_int = static_cast<int>(std::round(projected_uv[i].y));
        visibility_buffer_.insert_depth(u_int, v_int, pts_thermal[i].z);
    }

    // 4. Sample temperatures with Bilinear Interpolation and Occlusion check
    int W = thermal_calib_.width;
    int H = thermal_calib_.height;

    for (size_t i = 0; i < n_points; ++i) {
        output[i].position_left = pts_left_optical[i];
        output[i].observation_dt_s = dt_s;
        output[i].temperature_c = -999.0f;
        output[i].confidence = 0.0f;
        output[i].valid = 0;

        if (!positive_depth[i]) continue;

        float u = projected_uv[i].x;
        float v = projected_uv[i].y;

        // Boundary check (strict: inside [0, W-1] and [0, H-1])
        if (u < 0.0f || u > static_cast<float>(W - 1) || v < 0.0f || v > static_cast<float>(H - 1)) {
            continue;
        }

        // Occlusion check against z-buffer
        int u_int = static_cast<int>(std::round(u));
        int v_int = static_cast<int>(std::round(v));
        if (!visibility_buffer_.is_visible(u_int, v_int, pts_thermal[i].z)) {
            continue; // Occluded by foreground surface
        }

        // Bilinear interpolation of temperature matrix
        int u0 = static_cast<int>(std::floor(u));
        int v0 = static_cast<int>(std::floor(v));
        int u1 = std::min(u0 + 1, W - 1);
        int v1 = std::min(v0 + 1, H - 1);

        float du = u - static_cast<float>(u0);
        float dv = v - static_cast<float>(v0);

        float t00 = thermal_matrix.at<float>(v0, u0);
        float t01 = thermal_matrix.at<float>(v0, u1);
        float t10 = thermal_matrix.at<float>(v1, u0);
        float t11 = thermal_matrix.at<float>(v1, u1);

        float sampled_temp = (1.0f - du) * (1.0f - dv) * t00 +
                             du * (1.0f - dv) * t01 +
                             (1.0f - du) * dv * t10 +
                             du * dv * t11;

        output[i].temperature_c = sampled_temp;
        output[i].confidence = std::max(0.0f, 1.0f - (dt_s / 0.1f)); // Confidence degrades with time skew
        output[i].valid = 1;
    }

    return output;
}

} // namespace mapping
} // namespace dron
