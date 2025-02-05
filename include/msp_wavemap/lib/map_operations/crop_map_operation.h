#ifndef WAVEMAP_MSP_MAP_OPERATIONS_CROP_MAP_OPERATION_H_
#define WAVEMAP_MSP_MAP_OPERATIONS_CROP_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <utility>

#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

namespace wavemap {
/**
 * Config struct for map cropping operations.
 */
struct CropMapOperationConfig : public ConfigBase<CropMapOperationConfig, 2> {
  //! Time period controlling how often the map is cropped.
  Seconds<FloatingPoint> once_every = 10.f;

  //! Distance beyond which blocks are deleted when the cropper is executed.
  Meters<FloatingPoint> remove_blocks_beyond_distance;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class CropMapOperation : public MapOperationBase {
 public:
  CropMapOperation(const CropMapOperationConfig& config,
                   MapBase::Ptr occupancy_map, Point3D* pos);

  bool shouldRun(const uint64_t current_time);

  void run(bool force_run) override;

 private:

  const CropMapOperationConfig config_;
  Point3D* pos_;
  uint64_t last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_CROP_MAP_OPERATION_H_