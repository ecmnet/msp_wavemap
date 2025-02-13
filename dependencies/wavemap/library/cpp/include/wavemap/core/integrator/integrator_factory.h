#ifndef WAVEMAP_CORE_INTEGRATOR_INTEGRATOR_FACTORY_H_
#define WAVEMAP_CORE_INTEGRATOR_INTEGRATOR_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/core/integrator/integrator_base.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap {
class IntegratorFactory {
 public:
  static std::unique_ptr<IntegratorBase> create(
      const param::Value& params, MapBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool = nullptr,
      std::optional<IntegratorType> default_integrator_type = std::nullopt);

  static std::unique_ptr<IntegratorBase> create(
      IntegratorType integrator_type, const param::Value& params,
      MapBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool = nullptr);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_INTEGRATOR_FACTORY_H_
