#ifndef WAVEMAP_CORE_UTILS_SDF_FULL_EUCLIDEAN_SDF_GENERATOR_H_
#define WAVEMAP_CORE_UTILS_SDF_FULL_EUCLIDEAN_SDF_GENERATOR_H_

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/bucket_queue.h"
#include "wavemap/core/map/hashed_blocks.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/utils/neighbors/grid_neighborhood.h"

namespace wavemap {
struct VectorDistance {
  Index3D parent;
  FloatingPoint distance;

  friend bool operator==(const VectorDistance& lhs, const VectorDistance& rhs) {
    return lhs.parent == rhs.parent && lhs.distance == rhs.distance;
  }
  friend bool operator!=(const VectorDistance& lhs, const VectorDistance& rhs) {
    return !(lhs == rhs);
  }
};

using VectorDistanceField = DenseBlockHash<VectorDistance, MapBase::kDim, 16>;

class FullEuclideanSDFGenerator {
 public:
  static constexpr FloatingPoint kMaxRelativeUnderEstimate = 1e-2f;
  static constexpr FloatingPoint kMaxRelativeOverEstimate = 1e-2f;

  explicit FullEuclideanSDFGenerator(FloatingPoint max_distance = 2.f,
                                     FloatingPoint occupancy_threshold = 0.f)
      : max_distance_(max_distance), classifier_(occupancy_threshold) {}

  HashedBlocks generate(const HashedWaveletOctree& occupancy_map) const;

  FloatingPoint getMaxDistance() const { return max_distance_; }

 private:
  inline static const auto kNeighborIndexOffsets =
      GridNeighborhood<3>::generateIndexOffsets<Adjacency::kAnyDisjoint>();

  const FloatingPoint max_distance_;
  const OccupancyClassifier classifier_;

  static FloatingPoint minDistanceTo(const Index3D& child,
                                     const Index3D& parent,
                                     FloatingPoint min_cell_width);

  void seed(const HashedWaveletOctree& occupancy_map, VectorDistanceField& sdf,
            BucketQueue<Index3D>& open_queue) const;
  void propagate(const HashedWaveletOctree& occupancy_map,
                 VectorDistanceField& sdf,
                 BucketQueue<Index3D>& open_queue) const;
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_SDF_FULL_EUCLIDEAN_SDF_GENERATOR_H_
