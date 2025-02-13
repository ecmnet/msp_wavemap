#include "pywavemap/pipeline.h"

#include <string>

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <wavemap/pipeline/pipeline.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_pipeline_bindings(nb::module_& m) {
  nb::class_<Pipeline>(m, "Pipeline",
                       "A class to build pipelines of measurement integrators "
                       "and map operations.")
      .def(nb::init<std::shared_ptr<MapBase>>(), "map"_a)
      .def("clear", &Pipeline::clear,
           "Deregister all the pipeline's measurement integrators and map "
           "operations.")
      .def("has_integrator", &Pipeline::hasIntegrator, "integrator_name"_a,
           "Returns true if an integrator with the given name has been "
           "registered.")
      .def("remove_integrator", &Pipeline::removeIntegrator,
           "integrator_name"_a,
           "Deregister the integrator with the given name. Returns true if it "
           "existed.")
      .def(
          "add_integrator",
          [](Pipeline& self, const std::string& integrator_name,
             const param::Value& params) -> bool {
            return self.addIntegrator(integrator_name, params);
          },
          nb::sig("def add_integrator(self, integrator_name: str, "
                  "integrator_params: dict) -> bool"),
          "integrator_name"_a, "integrator_params"_a,
          "Create and register a new integrator")
      .def("clear_integrators", &Pipeline::clearIntegrators,
           "Deregister all integrators.")
      .def(
          "add_operation",
          [](Pipeline& self, const param::Value& params) -> bool {
            return self.addOperation(params);
          },
          nb::sig("def add_operation(self, operation_params: dict) -> bool"),
          "operation_params"_a, "Create and register a new map operation.")
      .def("clear_operations", &Pipeline::clearOperations,
           "Deregister all map operations")
      .def("run_integrators", &Pipeline::runIntegrators<PosedPointcloud<>>,
           "integrator_names"_a, "posed_pointcloud"_a,
           "Integrate a given pointcloud.")
      .def("run_integrators", &Pipeline::runIntegrators<PosedImage<>>,
           "integrator_names"_a, "posed_image"_a,
           "Integrate a given depth image.")
      .def("run_operations", &Pipeline::runOperations, "force_run_all"_a,
           "Run the map operations.")
      .def("run_pipeline", &Pipeline::runPipeline<PosedPointcloud<>>,
           "integrator_names"_a, "posed_pointcloud"_a,
           "Integrate a given pointcloud, then run the map operations.")
      .def("run_pipeline", &Pipeline::runPipeline<PosedImage<>>,
           "integrator_names"_a, "posed_image"_a,
           "Integrate a given depth image, then run the map operations.");
}
}  // namespace wavemap
