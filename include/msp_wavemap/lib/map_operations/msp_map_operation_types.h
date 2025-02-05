#ifndef WAVEMAP_MSP_MAP_OPERATIONS_MAP_MSP_OPERATION_TYPES_H_
#define WAVEMAP_MSP_MAP_OPERATIONS_MAP_MSP_OPERATION_TYPES_H_

#include <wavemap/core/config/type_selector.h>

namespace wavemap {
struct MapMSPOperationType : public TypeSelector<MapMSPOperationType> {
  using TypeSelector<MapMSPOperationType>::TypeSelector;

  enum Id : TypeId { kCropMap };

  static constexpr std::array names = { "crop_map"};
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_MAP_MSP_OPERATION_TYPES_H_