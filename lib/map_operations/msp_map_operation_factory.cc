#include "msp_wavemap/lib/map_operations/msp_map_operation_factory.h"

#include <memory>
#include <string>
#include <iostream>
#include <utility>

#include "msp_wavemap/lib/map_operations/crop_map_operation.h"


namespace wavemap {
std::unique_ptr<MapOperationBase> MapMSPOperationFactory::create(
    const param::Value& params, MapBase::Ptr occupancy_map, Point3D* pos,
    std::shared_ptr<ThreadPool> thread_pool) {
  if (const auto type = MapMSPOperationType::from(params); type) {
    return create(type.value(), params, std::move(occupancy_map), pos,
                  std::move(thread_pool));
  }
   std::cout << "Could not create MSP map operation" << std::endl;
  return nullptr;
}

std::unique_ptr<MapOperationBase> MapMSPOperationFactory::create(
    MapMSPOperationType ros_operation_type, const param::Value& params,
    MapBase::Ptr occupancy_map, Point3D* pos, std::shared_ptr<ThreadPool> thread_pool) {
  if (!ros_operation_type.isValid()) {
    std::cout << "Received request to create map operation with invalid type." << std::endl;
    return nullptr;
  }

  // Create the operation handler
  switch (ros_operation_type) {
    case MapMSPOperationType::kCropMap:
      if (const auto config = CropMapOperationConfig::from(params); config) {
        return std::make_unique<CropMapOperation>(
            config.value(), std::move(occupancy_map), pos);
      } else {
        std::cout << "Crop map operation config could not be loaded." << std::endl;
        return nullptr;
      }
  }

 std::cout << "Factory does not (yet) support creation of map operation type " 
             << ros_operation_type.toStr() << "." << std::endl;
  return nullptr;
}
}  // namespace wavemap