#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap/core/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/core/integrator/projective/projective_integrator.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/map/wavelet_octree.h"

namespace wavemap {
class WaveletIntegrator : public ProjectiveIntegrator {
 public:
  WaveletIntegrator(const ProjectiveIntegratorConfig& config,
                    ProjectorBase::ConstPtr projection_model,
                    PosedImage<>::Ptr posed_range_image,
                    Image<Vector2D>::Ptr beam_offset_image,
                    MeasurementModelBase::ConstPtr measurement_model,
                    WaveletOctree::Ptr occupancy_map)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))),
        min_cell_width_(occupancy_map_->getMinCellWidth()) {}

 private:
  const WaveletOctree::Ptr occupancy_map_;
  std::shared_ptr<RangeImageIntersector> range_image_intersector_;

  // Cache/pre-computed commonly used values
  const FloatingPoint min_cell_width_;
  const IndexElement termination_height_ =
      min_cell_width_ < config_.max_update_resolution
          ? static_cast<IndexElement>(std::round(
                std::log2(config_.max_update_resolution / min_cell_width_)))
          : 0;
  static constexpr auto kUnitCubeHalfDiagonal =
      constants<FloatingPoint>::kSqrt3 / 2.f;

  WaveletOctree::Coefficients::Scale recursiveSamplerCompressor(
      const OctreeIndex& node_index, FloatingPoint node_value,
      WaveletOctree::NodeType& parent_node,
      OctreeIndex ::RelativeChild relative_child_index);

  void updateMap() override;
};
}  // namespace wavemap

#include "wavemap/core/integrator/projective/coarse_to_fine/impl/wavelet_integrator_inl.h"

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
