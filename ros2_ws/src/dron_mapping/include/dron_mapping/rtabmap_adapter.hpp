#pragma once

#include "observation_store.hpp"
#include <vector>
#include <string>

namespace dron {
namespace mapping {

class RtabmapAdapter {
public:
    RtabmapAdapter(ObservationStore& store);

    /**
     * @brief Process map graph updates from RTAB-Map
     * @param node_ids List of node/keyframe IDs from MapGraph.poses_id
     * @param poses_matrices List of 4x4 optimized poses
     * @param revision Increasing revision counter
     */
    void handle_graph_update(const std::vector<uint64_t>& node_ids,
                             const std::vector<cv::Mat>& poses_matrices,
                             uint64_t revision);

    void handle_map_reset();

private:
    ObservationStore& store_;
};

} // namespace mapping
} // namespace dron
