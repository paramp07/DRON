#include "dron_mapping/rtabmap_adapter.hpp"

namespace dron {
namespace mapping {

RtabmapAdapter::RtabmapAdapter(ObservationStore& store) : store_(store) {}

void RtabmapAdapter::handle_graph_update(const std::vector<uint64_t>& node_ids,
                                         const std::vector<cv::Mat>& poses_matrices,
                                         uint64_t revision) {
    store_.update_poses(node_ids, poses_matrices, revision);
}

void RtabmapAdapter::handle_map_reset() {
    store_.reset();
}

} // namespace mapping
} // namespace dron
