#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap/core/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/core/integrator/projective/projective_integrator.h"
#include "wavemap/core/integrator/projective/update_type.h"
#include "wavemap/core/map/volumetric_octree.h"

namespace wavemap {
class CoarseToFineIntegrator : public ProjectiveIntegrator {
 public:
  CoarseToFineIntegrator(const ProjectiveIntegratorConfig& config,
                         ProjectorBase::ConstPtr projection_model,
                         PosedImage<>::Ptr posed_range_image,
                         Image<Vector2D>::Ptr beam_offset_image,
                         MeasurementModelBase::ConstPtr measurement_model,
                         VolumetricOctree::Ptr occupancy_map)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))),
        min_cell_width_(occupancy_map_->getMinCellWidth()) {}

 private:
  const VolumetricOctree::Ptr occupancy_map_;
  const FloatingPoint min_cell_width_;

  std::shared_ptr<RangeImageIntersector> range_image_intersector_;
  static constexpr auto kUnitCubeHalfDiagonal =
      constants<FloatingPoint>::kSqrt3 / 2.f;

  void updateMap() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
