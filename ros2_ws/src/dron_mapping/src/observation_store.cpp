#include "dron_mapping/observation_store.hpp"

namespace dron {
namespace mapping {

ObservationStore::ObservationStore() : current_revision_(0) {}

void ObservationStore::add_chunk(const KeyframeChunk& chunk) {
    std::lock_guard<std::mutex> lock(mutex_);
    chunks_[chunk.chunk_id] = chunk;
}

bool ObservationStore::get_chunk(uint64_t chunk_id, KeyframeChunk& out_chunk) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = chunks_.find(chunk_id);
    if (it != chunks_.end()) {
        out_chunk = it->second;
        return true;
    }
    return false;
}

void ObservationStore::update_poses(const std::vector<uint64_t>& chunk_ids,
                                    const std::vector<cv::Mat>& poses,
                                    uint64_t revision) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (revision <= current_revision_) {
        return; // Ignore stale revisions
    }
    current_revision_ = revision;

    for (size_t i = 0; i < chunk_ids.size() && i < poses.size(); ++i) {
        auto it = chunks_.find(chunk_ids[i]);
        if (it != chunks_.end()) {
            it->second.pose_world_keyframe = poses[i].clone();
            it->second.revision = revision;
        }
    }
}

void ObservationStore::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    chunks_.clear();
    current_revision_ = 0;
}

std::vector<KeyframeChunk> ObservationStore::get_all_chunks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<KeyframeChunk> result;
    result.reserve(chunks_.size());
    for (const auto& pair : chunks_) {
        result.push_back(pair.second);
    }
    return result;
}

size_t ObservationStore::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return chunks_.size();
}

} // namespace mapping
} // namespace dron
