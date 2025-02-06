#include "msp_wavemap/lib/map_operations/crop_map_operation.h"

#include <memory>
#include <string>
#include <iostream>
#include <utility>

#include <wavemap/core/map/hashed_blocks.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

//TODO: Crop based on movement not on time

namespace wavemap {
DECLARE_CONFIG_MEMBERS(CropMapOperationConfig,
                      (once_every)
                      (remove_blocks_beyond_distance)
                      (only_when_moved));

bool CropMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_GT(remove_blocks_beyond_distance, 0.f, verbose);

  return all_valid;
}

CropMapOperation::CropMapOperation(const CropMapOperationConfig& config,
                                   MapBase::Ptr occupancy_map, Point3D* pos)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()
    ) { pos_ = pos; last_pos_ = *pos; }

bool CropMapOperation::shouldRun(const uint64_t current_time) {
  if(config_.only_when_moved < 0)
    return false;
  return  ( 
            config_.only_when_moved == 0 || 
            ( (*pos_ - last_pos_).norm() > config_.only_when_moved ) 
          ) &&
    config_.once_every < (current_time - last_run_timestamp_);
}

void CropMapOperation::run(bool force_run) {
  const uint64_t current_time = std::chrono::system_clock::to_time_t(std::chrono::high_resolution_clock::now());
  if (!force_run && !shouldRun(current_time)) {
    return;
  }
  last_run_timestamp_ = current_time;
  last_pos_ = *pos_;

  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  const IndexElement tree_height = occupancy_map_->getTreeHeight();
  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const Point3D t_W_B = *pos_;

  auto indicator_fn = [tree_height, min_cell_width, &config = config_, &t_W_B](
                          const Index3D& block_index, const auto& /*block*/) {
    const auto block_node_index = OctreeIndex{tree_height, block_index};
    const auto block_aabb =
        convert::nodeIndexToAABB(block_node_index, min_cell_width);
    const FloatingPoint d_B_block = block_aabb.minDistanceTo(t_W_B);
    return config.remove_blocks_beyond_distance < d_B_block;
  };

  if (auto* hashed_wavelet_octree =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map_.get());
      hashed_wavelet_octree) {
    hashed_wavelet_octree->eraseBlockIf(indicator_fn);
  } else if (auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map_.get());
             hashed_chunked_wavelet_octree) {
    hashed_chunked_wavelet_octree->eraseBlockIf(indicator_fn);
  } else {
       std::cout << "Map cropping is only supported for hash-based map data structures." << std::endl;
  }
}
}  // namespace wavemap