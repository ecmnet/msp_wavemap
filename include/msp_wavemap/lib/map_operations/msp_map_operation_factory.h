#ifndef WAVEMAP_MSP_MAP_OPERATIONS_MAP_MSP_OPERATION_FACTORY_H_
#define WAVEMAP_MSP_MAP_OPERATIONS_MAP_MSP_OPERATION_FACTORY_H_

#include <memory>
#include <string>

#include <wavemap/core/config/param.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

#include "msp_wavemap/lib/map_operations/msp_map_operation_types.h"

namespace wavemap {
class MapMSPOperationFactory {
 public:
  static std::unique_ptr<MapOperationBase> create(
      const param::Value& params, MapBase::Ptr occupancy_map, Point3D* pos,
      std::shared_ptr<ThreadPool> thread_pool
      );

  static std::unique_ptr<MapOperationBase> create(
      MapMSPOperationType ros_operation_type, const param::Value& params,
      MapBase::Ptr occupancy_map, Point3D* pos, std::shared_ptr<ThreadPool> thread_pool
      );
};
}  // namespace wavemap

#endif  // WAVEMAP_MSP_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_