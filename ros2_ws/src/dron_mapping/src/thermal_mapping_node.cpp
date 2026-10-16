#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include "dron_mapping/thermal_projector.hpp"
#include "dron_mapping/observation_store.hpp"
#include "dron_mapping/rtabmap_adapter.hpp"
#include "dron_interfaces/msg/thermal_chunk.hpp"
#include "dron_interfaces/msg/mapping_status.hpp"

using namespace std::chrono_literals;

namespace dron {
namespace mapping {

class ThermalMappingNode : public rclcpp::Node {
public:
    ThermalMappingNode() : Node("thermal_mapping_node") {
        // Load parameters
        this->declare_parameter("depth_tolerance_m", 0.15);
        this->declare_parameter("session_id", "session_001");
        this->declare_parameter("drone_id", "drone_01");
        
        session_id_ = this->get_parameter("session_id").as_string();
        drone_id_ = this->get_parameter("drone_id").as_string();
        float tol = static_cast<float>(this->get_parameter("depth_tolerance_m").as_double());

        // Initialize calibration structs (default test rig)
        thermal_calib_.width = 80;
        thermal_calib_.height = 62;
        thermal_calib_.K = (cv::Mat_<double>(3,3) << 96.5, 0.0, 39.5, 0.0, 96.5, 30.5, 0.0, 0.0, 1.0);
        thermal_calib_.D = (cv::Mat_<double>(1,5) << -0.05, 0.01, 0.0, 0.0, 0.0);
        
        rig_extrinsics_.R_thermal_left = cv::Mat::eye(3, 3, CV_64F);
        rig_extrinsics_.T_thermal_left = (cv::Mat_<double>(3,1) << -0.040, -0.025, 0.005);
        rig_extrinsics_.calibration_id = "calib_20261016_v1";

        projector_ = std::make_unique<ThermalProjector>(thermal_calib_, rig_extrinsics_, tol);

        // Publishers
        pub_chunk_ = this->create_publisher<dron_interfaces::msg::ThermalChunk>("mapping/keyframe_chunk", 10);
        pub_preview_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("thermal/points_preview", 10);
        pub_status_ = this->create_publisher<dron_interfaces::msg::MappingStatus>("mapping/status", 10);

        RCLCPP_INFO(this->get_logger(), "ThermalMappingNode initialized for drone '%s', session '%s'.", drone_id_.c_str(), session_id_.c_str());
    }

private:
    std::string session_id_;
    std::string drone_id_;
    CameraIntrinsics thermal_calib_;
    RigExtrinsics rig_extrinsics_;
    std::unique_ptr<ThermalProjector> projector_;
    ObservationStore store_;

    rclcpp::Publisher<dron_interfaces::msg::ThermalChunk>::SharedPtr pub_chunk_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_preview_;
    rclcpp::Publisher<dron_interfaces::msg::MappingStatus>::SharedPtr pub_status_;
};

} // namespace mapping
} // namespace dron

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dron::mapping::ThermalMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
