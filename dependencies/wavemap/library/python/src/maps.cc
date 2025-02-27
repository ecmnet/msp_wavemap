#include "pywavemap/maps.h"

#include <memory>
#include <vector>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/shared_ptr.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/query/map_interpolator.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/io/file_conversions.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_map_bindings(nb::module_& m) {
  enum class InterpolationMode { kNearest, kTrilinear };

  nb::enum_<InterpolationMode>(m, "InterpolationMode")
      .value("NEAREST", InterpolationMode::kNearest,
             "Look up the value of the nearest map cell.")
      .value("TRILINEAR", InterpolationMode::kTrilinear,
             "Interpolate linearly along each map axis.");

  nb::class_<MapBase>(m, "Map", "Base class for wavemap maps.")
      .def_prop_ro("empty", &MapBase::empty, "Whether the map is empty.")
      .def_prop_ro("size", &MapBase::size,
                   "The number of cells or nodes in the map, for fixed or "
                   "multi-resolution maps, respectively.")
      .def("threshold", &MapBase::threshold,
           "Threshold the occupancy values of all cells in the map to stay "
           "within the range specified by its min_log_odds and max_log_odds.")
      .def("prune", &MapBase::prune,
           "Free up memory by pruning nodes that are no longer needed. Note "
           "that this pruning operation is lossless and does not alter the "
           "estimated occupancy posterior.")
      .def("prune_smart", &MapBase::pruneSmart,
           "Similar to prune(), but avoids de-allocating nodes that were "
           "recently updated and will likely be used again in the near future.")
      .def("clear", &MapBase::clear, "Erase all cells in the map.")
      .def_prop_ro("min_cell_width", &MapBase::getMinCellWidth,
                   "Maximum map resolution, set as width of smallest cell it "
                   "can represent.")
      .def_prop_ro("min_log_odds", &MapBase::getMinLogOdds,
                   "Lower threshold for the occupancy values stored in the "
                   "map, in log-odds.")
      .def_prop_ro("max_log_odds", &MapBase::getMaxLogOdds,
                   "Upper threshold for the occupancy values stored in the "
                   "map, in log-odds.")
      .def_prop_ro("memory_usage", &MapBase::getMemoryUsage,
                   "The amount of memory used by the map, in bytes.")
      .def_prop_ro("tree_height", &MapBase::getTreeHeight,
                   "Height of the octree used to store the map. Note that this "
                   "value is only defined for multi-resolution maps.")
      .def_prop_ro("min_index", &MapBase::getMinIndex,
                   "Index of the minimum corner of the map's Axis Aligned "
                   "Bounding Box.")
      .def_prop_ro("max_index", &MapBase::getMaxIndex,
                   "Index of the maximum corner of the map's Axis Aligned "
                   "Bounding Box.")
      .def("get_cell_value", &MapBase::getCellValue, "index"_a,
           "Query the value of the map at a given index.")
      .def("set_cell_value", &MapBase::setCellValue, "index"_a,
           "new_value"_a
           "Set the value of the map at a given index.")
      .def("add_to_cell_value", &MapBase::addToCellValue, "index"_a, "update"_a,
           "Increment the value of the map at a given index.")
      .def(
          "interpolate",
          [](const MapBase& self, const Point3D& position,
             InterpolationMode mode) {
            switch (mode) {
              case InterpolationMode::kNearest:
                return interpolate::nearestNeighbor(self, position);
              case InterpolationMode::kTrilinear:
                return interpolate::trilinear(self, position);
              default:
                throw nb::type_error("Unknown interpolation mode.");
            }
          },
          "position"_a, "mode"_a = InterpolationMode::kTrilinear,
          "Query the map's value at a point, using the specified interpolation "
          "mode.")
      .def_static(
          "create",
          [](const param::Value& params) -> std::shared_ptr<MapBase> {
            return MapFactory::create(params);
          },
          nb::sig("def create(parameters: dict) -> Map"), "parameters"_a,
          "Create a new map based on the given settings.")
      .def_static(
          "load",
          [](const std::filesystem::path& file_path)
              -> std::shared_ptr<MapBase> {
            std::shared_ptr<MapBase> map;
            if (io::fileToMap(file_path, map)) {
              return map;
            }
            return nullptr;
          },
          "file_path"_a, "Load a wavemap map from a .wvmp file.")
      .def(
          "store",
          [](const MapBase& self, const std::filesystem::path& file_path)
              -> bool { return io::mapToFile(self, file_path); },
          "file_path"_a, "Store a wavemap map as a .wvmp file.")
      .def(
          "get_occupied_node_indices",
          [](const MapBase& self, FloatingPoint log_odds_occupancy_threshold) {
            // Get the node indices
            std::vector<OctreeIndex> node_indices;
            self.forEachLeaf(
                [&node_indices, log_odds_occupancy_threshold](
                    const OctreeIndex& node_index, FloatingPoint node_value) {
                  if (log_odds_occupancy_threshold < node_value) {
                    node_indices.emplace_back(node_index);
                  }
                });

            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new IndexElement[4 * node_indices.size()];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });

            // Populate the results
            size_t idx = 0;
            for (const auto& node_index : node_indices) {
              results[idx + 0] = node_index.height;
              results[idx + 1] = node_index.position.x();
              results[idx + 2] = node_index.position.y();
              results[idx + 3] = node_index.position.z();
              idx += 4;
            }

            // Return results as numpy array
            return nb::ndarray<nb::numpy, IndexElement>{
                results, {node_indices.size(), 4u}, owner};
          },
          "threshold"_a = 1e-3f,
          "Retrieve the indices of all occupied leaf nodes.\n\n"
          "    :param threshold: The log-odds threshold above which a node is "
          "considered occupied.\n"
          "    :returns: An (N, 4) numpy array where each row contains the "
          "height, x, y, and z indices (int32) of an occupied leaf node.")
      .def(
          "get_occupied_pointcloud",
          [](const MapBase& self, FloatingPoint log_odds_occupancy_threshold) {
            // Get the center points of all occupied high resolution cells
            std::vector<Point3D> pointcloud;
            const FloatingPoint min_cell_width = self.getMinCellWidth();
            self.forEachLeaf(
                [&pointcloud, log_odds_occupancy_threshold, min_cell_width](
                    const OctreeIndex& node_index, FloatingPoint node_value) {
                  if (log_odds_occupancy_threshold < node_value) {
                    if (node_index.height == 0) {
                      const Point3D center = convert::indexToCenterPoint(
                          node_index.position, min_cell_width);
                      pointcloud.emplace_back(center);
                    } else {
                      const Index3D node_min_corner =
                          convert::nodeIndexToMinCornerIndex(node_index);
                      const Index3D node_max_corner =
                          convert::nodeIndexToMaxCornerIndex(node_index);
                      for (const Index3D& index :
                           Grid(node_min_corner, node_max_corner)) {
                        const Point3D center =
                            convert::indexToCenterPoint(index, min_cell_width);
                        pointcloud.emplace_back(center);
                      }
                    }
                  }
                });

            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[3 * pointcloud.size()];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });

            // Populate the results
            size_t idx = 0;
            for (const auto& point : pointcloud) {
              results[idx + 0] = point.x();
              results[idx + 1] = point.y();
              results[idx + 2] = point.z();
              idx += 3;
            }

            // Return results as numpy array
            return nb::ndarray<nb::numpy, FloatingPoint>{
                results, {pointcloud.size(), 3u}, owner};
          },
          "threshold"_a = 1e-3f,
          "Retrieve the center points of all occupied cells at the highest "
          "resolution.\n\n"
          "    :param threshold: The log-odds threshold above which a cell is "
          "considered occupied.\n"
          "    :returns: An (N, 3) numpy array where each row contains the "
          "x, y, and z coordinates (float32) of an occupied cell center.");

  nb::class_<HashedWaveletOctree, MapBase>(
      m, "HashedWaveletOctree",
      "A class that stores maps using hashed wavelet octrees.")
      .def("get_cell_value", &MapBase::getCellValue, "index"_a,
           "Query the value of the map at a given index.")
      .def("get_cell_value",
           nb::overload_cast<const OctreeIndex&>(
               &HashedWaveletOctree::getCellValue, nb::const_),
           "node_index"_a,
           "Query the value of the map at a given octree node index.")
      .def(
          "get_cell_values",
          [](const HashedWaveletOctree& self,
             const nb::ndarray<IndexElement, nb::shape<-1, 3>, nb::device::cpu>&
                 indices) {
            // Create a query accelerator
            QueryAccelerator<HashedWaveletOctree> query_accelerator{self};
            // Create nb::ndarray view for efficient access to the query indices
            const auto index_view = indices.view();
            const auto num_queries = index_view.shape(0);
            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[num_queries];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Compute the interpolated values
            for (size_t query_idx = 0; query_idx < num_queries; ++query_idx) {
              results[query_idx] = query_accelerator.getCellValue(
                  {index_view(query_idx, 0), index_view(query_idx, 1),
                   index_view(query_idx, 2)});
            }
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "index_list"_a,
          "Query the map at the given indices, provided as a matrix with one "
          "(x, y, z) index per row.")
      .def(
          "get_cell_values",
          [](const HashedWaveletOctree& self,
             const nb::ndarray<IndexElement, nb::shape<-1, 4>, nb::device::cpu>&
                 indices) {
            // Create a query accelerator
            QueryAccelerator<HashedWaveletOctree> query_accelerator{self};
            // Create nb::ndarray view for efficient access to the query indices
            auto index_view = indices.view();
            const auto num_queries = index_view.shape(0);
            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[num_queries];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Compute the interpolated values
            for (size_t query_idx = 0; query_idx < num_queries; ++query_idx) {
              const OctreeIndex node_index{
                  index_view(query_idx, 0),
                  {index_view(query_idx, 1), index_view(query_idx, 2),
                   index_view(query_idx, 3)}};
              results[query_idx] = query_accelerator.getCellValue(node_index);
            }
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "node_index_list"_a,
          "Query the map at the given node indices, provided as a matrix with "
          "one (height, x, y, z) node index per row.")
      .def(
          "interpolate",
          [](const MapBase& self, const Point3D& position,
             InterpolationMode mode) {
            switch (mode) {
              case InterpolationMode::kNearest:
                return interpolate::nearestNeighbor(self, position);
              case InterpolationMode::kTrilinear:
                return interpolate::trilinear(self, position);
              default:
                throw nb::type_error("Unknown interpolation mode.");
            }
          },
          "position"_a, "mode"_a = InterpolationMode::kTrilinear,
          "Query the map's value at a point, using the specified interpolation "
          "mode.")
      .def(
          "interpolate",
          [](const HashedWaveletOctree& self,
             const nb::ndarray<FloatingPoint, nb::shape<-1, 3>,
                               nb::device::cpu>& positions,
             InterpolationMode mode) {
            // Create a query accelerator
            QueryAccelerator<HashedWaveletOctree> query_accelerator{self};
            // Create nb::ndarray view for efficient access to the query points
            const auto positions_view = positions.view();
            const auto num_queries = positions_view.shape(0);
            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[num_queries];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Compute the interpolated values
            switch (mode) {
              case InterpolationMode::kNearest:
                for (size_t query_idx = 0; query_idx < num_queries;
                     ++query_idx) {
                  results[query_idx] = interpolate::nearestNeighbor(
                      query_accelerator, {positions_view(query_idx, 0),
                                          positions_view(query_idx, 1),
                                          positions_view(query_idx, 2)});
                }
                break;
              case InterpolationMode::kTrilinear:
                for (size_t query_idx = 0; query_idx < num_queries;
                     ++query_idx) {
                  results[query_idx] = interpolate::trilinear(
                      query_accelerator, {positions_view(query_idx, 0),
                                          positions_view(query_idx, 1),
                                          positions_view(query_idx, 2)});
                }
                break;
              default:
                throw nb::type_error("Unknown interpolation mode.");
            }
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "position_list"_a, "mode"_a = InterpolationMode::kTrilinear,
          "Query the map's value at the given points, using the specified "
          "interpolation mode.");

  nb::class_<HashedChunkedWaveletOctree, MapBase>(
      m, "HashedChunkedWaveletOctree",
      "A class that stores maps using hashed chunked wavelet octrees.")
      .def("get_cell_value", &MapBase::getCellValue, "index"_a,
           "Query the value of the map at a given index.")
      .def("get_cell_value",
           nb::overload_cast<const OctreeIndex&>(
               &HashedChunkedWaveletOctree::getCellValue, nb::const_),
           "node_index"_a,
           "Query the value of the map at a given octree node index.")
      .def(
          "get_cell_values",
          [](const HashedChunkedWaveletOctree& self,
             const nb::ndarray<IndexElement, nb::shape<-1, 3>, nb::device::cpu>&
                 indices) {
            // Create a query accelerator
            QueryAccelerator<HashedChunkedWaveletOctree> query_accelerator{
                self};
            // Create nb::ndarray view for efficient access to the query indices
            const auto index_view = indices.view();
            const auto num_queries = index_view.shape(0);
            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[num_queries];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Compute the interpolated values
            for (size_t query_idx = 0; query_idx < num_queries; ++query_idx) {
              results[query_idx] = query_accelerator.getCellValue(
                  {index_view(query_idx, 0), index_view(query_idx, 1),
                   index_view(query_idx, 2)});
            }
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "index_list"_a,
          "Query the map at the given indices, provided as a matrix with one "
          "(x, y, z) index per row.")
      .def(
          "get_cell_values",
          [](const HashedChunkedWaveletOctree& self,
             const nb::ndarray<IndexElement, nb::shape<-1, 4>, nb::device::cpu>&
                 indices) {
            // Create a query accelerator
            QueryAccelerator<HashedChunkedWaveletOctree> query_accelerator{
                self};
            // Create nb::ndarray view for efficient access to the query indices
            auto index_view = indices.view();
            const auto num_queries = index_view.shape(0);
            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[num_queries];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Compute the interpolated values
            for (size_t query_idx = 0; query_idx < num_queries; ++query_idx) {
              const OctreeIndex node_index{
                  index_view(query_idx, 0),
                  {index_view(query_idx, 1), index_view(query_idx, 2),
                   index_view(query_idx, 3)}};
              results[query_idx] = query_accelerator.getCellValue(node_index);
            }
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "node_index_list"_a,
          "Query the map at the given node indices, provided as a matrix with "
          "one (height, x, y, z) node index per row.")
      .def(
          "interpolate",
          [](const MapBase& self, const Point3D& position,
             InterpolationMode mode) {
            switch (mode) {
              case InterpolationMode::kNearest:
                return interpolate::nearestNeighbor(self, position);
              case InterpolationMode::kTrilinear:
                return interpolate::trilinear(self, position);
              default:
                throw nb::type_error("Unknown interpolation mode.");
            }
          },
          "position"_a, "mode"_a = InterpolationMode::kTrilinear,
          "Query the map's value at a point, using the specified interpolation "
          "mode.")
      .def(
          "interpolate",
          [](const HashedChunkedWaveletOctree& self,
             const nb::ndarray<FloatingPoint, nb::shape<-1, 3>,
                               nb::device::cpu>& positions,
             InterpolationMode mode) {
            // Create a query accelerator
            QueryAccelerator<HashedChunkedWaveletOctree> query_accelerator{
                self};
            // Create nb::ndarray view for efficient access to the query points
            const auto positions_view = positions.view();
            const auto num_queries = positions_view.shape(0);
            // Create the raw results array and wrap it in a Python capsule that
            // deallocates it when all references to it expire
            auto* results = new float[num_queries];
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Compute the interpolated values
            switch (mode) {
              case InterpolationMode::kNearest:
                for (size_t query_idx = 0; query_idx < num_queries;
                     ++query_idx) {
                  results[query_idx] = interpolate::nearestNeighbor(
                      query_accelerator, {positions_view(query_idx, 0),
                                          positions_view(query_idx, 1),
                                          positions_view(query_idx, 2)});
                }
                break;
              case InterpolationMode::kTrilinear:
                for (size_t query_idx = 0; query_idx < num_queries;
                     ++query_idx) {
                  results[query_idx] = interpolate::trilinear(
                      query_accelerator, {positions_view(query_idx, 0),
                                          positions_view(query_idx, 1),
                                          positions_view(query_idx, 2)});
                }
                break;
              default:
                throw nb::type_error("Unknown interpolation mode.");
            }
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "position_list"_a, "mode"_a = InterpolationMode::kTrilinear,
          "Query the map's value at the given points, using the specified "
          "interpolation mode.");
}
}  // namespace wavemap
