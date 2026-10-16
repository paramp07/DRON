#pragma once

#include "thermal_projector.hpp"
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <memory>

namespace dron {
namespace mapping {

struct KeyframeChunk {
    uint64_t chunk_id;
    uint64_t revision;
    double timestamp_s;
    std::string session_id;
    std::string drone_id;
    cv::Mat pose_world_keyframe; // 4x4 matrix
    std::vector<ProjectedThermalPoint> points;
};

class ObservationStore {
public:
    ObservationStore();

    void add_chunk(const KeyframeChunk& chunk);
    bool get_chunk(uint64_t chunk_id, KeyframeChunk& out_chunk) const;
    void update_poses(const std::vector<uint64_t>& chunk_ids,
                      const std::vector<cv::Mat>& poses,
                      uint64_t revision);
    void reset();
    std::vector<KeyframeChunk> get_all_chunks() const;
    size_t size() const;

private:
    mutable std::mutex mutex_;
    std::map<uint64_t, KeyframeChunk> chunks_;
    uint64_t current_revision_ = 0;
};

} // namespace mapping
} // namespace dron
