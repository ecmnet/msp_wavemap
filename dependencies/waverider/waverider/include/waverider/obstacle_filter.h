#ifndef WAVERIDER_OBSTACLE_FILTER_H_
#define WAVERIDER_OBSTACLE_FILTER_H_

#include <algorithm>
#include <atomic>
#include <utility>
#include <vector>
#include <mutex>

#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/cell_types/haar_coefficients.h>


#include "waverider/common.h"

using namespace wavemap;

static constexpr int kDim = 3;
  using BlockIndex = Index3D;
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using OctreeType = Octree<Coefficients::Details>;

namespace waverider {
struct ObstacleCells {
  std::vector<std::vector<Eigen::Vector3f>> centers;
  std::vector<float> cell_widths;

  void swap(ObstacleCells& other);
};

class WavemapObstacleFilter {
 public:
  WavemapObstacleFilter() = default;

  void setOccupancyThreshold(FloatingPoint threshold) {
    occupancy_threshold_ = threshold;
  }

  void update(const wavemap::HashedWaveletOctree& map,
              const Point3D& robot_position, const Plane3D& ground_plane);

  bool isReady() const {
    return !obstacle_cells_.centers.empty() ||
           new_obstacle_cells_.ready.load(std::memory_order_relaxed);
  }

  // WARNING: This method also is only safe to call from a single thread,
  //          as it also swaps in new the obstacles when available.
  const ObstacleCells& getObstacleCells();

  bool use_only_lowest_level_ = false;
  double lowest_level_radius_{3.0};
  static FloatingPoint maxRangeForHeight(int level) {
    return std::pow(3.f, static_cast<FloatingPoint>(level) / 3.f) - 0.25f;
  }

 private:
  using HashedWaveletOctreeBlock = wavemap::HashedWaveletOctreeBlock;

  size_t function_evals_ = 0;
  FloatingPoint occupancy_threshold_ = -0.1;

  FloatingPoint min_cell_width_ = wavemap::kNaN;
  FloatingPoint min_log_odds_shrunk_ = wavemap::kNaN;
  int tree_height_ = -1;

  ObstacleCells obstacle_cells_;
  struct {
    ObstacleCells data;
    std::mutex mutex;
    std::atomic<bool> ready{false};
  } new_obstacle_cells_;

  // function that defines the radius we care about for each tree level

  int minHeightForRange(FloatingPoint range) const {
    const FloatingPoint range_clamped = std::max(range, 1.f);
    return std::clamp(2 * static_cast<int>(std::log(range_clamped)) - 1, 0,
                      tree_height_);
  }

  void leafObstacleFilter(
      const Point3D& robot_position, const Plane3D& ground_plane,
      const HashedWaveletOctreeBlock::BlockIndex& block_index,
      const HashedWaveletOctreeBlock& block);

  void adaptiveObstacleFilter(const Point3D& robot_position,
                              const Plane3D& ground_plane,
                              const OctreeIndex& node_index,
                              const OctreeType::NodeType& node,
                        
                              FloatingPoint node_occupancy);

  bool nodeHasOccupiedChild(const OctreeType::NodeType& node,
                            FloatingPoint node_occupancy);
};
}  // namespace waverider

#endif  // WAVERIDER_OBSTACLE_FILTER_H_
