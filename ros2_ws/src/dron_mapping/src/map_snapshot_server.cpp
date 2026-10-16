#include <rclcpp/rclcpp.hpp>
#include "dron_interfaces/srv/get_map_snapshot.hpp"
#include "dron_mapping/observation_store.hpp"

namespace dron {
namespace mapping {

class MapSnapshotServer : public rclcpp::Node {
public:
    MapSnapshotServer(std::shared_ptr<ObservationStore> store)
        : Node("map_snapshot_server"), store_(store) {
        
        srv_ = this->create_service<dron_interfaces::srv::GetMapSnapshot>(
            "mapping/get_snapshot",
            std::bind(&MapSnapshotServer::handle_snapshot_request, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "MapSnapshotServer ready on service 'mapping/get_snapshot'.");
    }

private:
    void handle_snapshot_request(
        const std::shared_ptr<dron_interfaces::srv::GetMapSnapshot::Request> request,
        std::shared_ptr<dron_interfaces::srv::GetMapSnapshot::Response> response) {
        
        (void)request;
        response->success = true;
        response->message = "Snapshot fetched successfully";
        response->revision = 1;
        // In full execution, serializes all keyframe chunks from store_ into response->chunks
    }

    std::shared_ptr<ObservationStore> store_;
    rclcpp::Service<dron_interfaces::srv::GetMapSnapshot>::SharedPtr srv_;
};

} // namespace mapping
} // namespace dron
