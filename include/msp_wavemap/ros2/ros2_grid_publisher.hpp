#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <msp_msgs/msg/micro_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>

#include "msp_controller/ros2/tf_transformer.h"

#include <algorithm>
#include <memory>

// TODO: Skip unchanged block as described in
// https : // github.com/ethz-asl/wavemap/blob/main/interfaces/ros1/wavemap_ros/src/map_operations/publish_pointcloud_operation.cc

using namespace wavemap;

namespace msp
{
    class MSPRos2GridPublisher
    {

    public:
        explicit MSPRos2GridPublisher(rclcpp::Node *node, std::shared_ptr<TfTransformer> transformer) : _node(node), transformer_(transformer)
        {

            microgrid_publisher_ = _node->create_publisher<msp_msgs::msg::MicroGrid>("msp/in/micro_grid", 100);

            timer_ = _node->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&MSPRos2GridPublisher::publish, this));
        }

        void setMap(wavemap::MapBase::Ptr map)
        {
            _map = map;
        }

    private:
        void publish()
        {
            const auto T_W_C = transformer_->lookupLatestTransform("world", "camera_link");

            if (!_map || T_W_C == std::nullopt)
                return;

            publish_grid(*T_W_C);
        }

        void transfer_grid(std::vector<Point3D> *list, Point3D r)
        {
             msp_msgs::msg::MicroGrid grid;

            if(list->size() == 0)
              return;

            int count = 0;

            grid.center[0] = r.x();
            grid.center[1] = r.y();
            grid.center[2] = r.z();
            grid.resolution = 0.2f;
            grid.extension = 10.0f;
            
            
            grid.count = 0;
            while (list->size() > 0)
            {
                auto p = list->back();
                list->pop_back();
                grid.data[grid.count] = encode(p, grid.extension, grid.resolution);
                if (grid.count >= 24)
                {   count++;
                   microgrid_publisher_->publish(grid);
                    grid.count = 0;
                    continue;
                }
                grid.count++;
            }

            microgrid_publisher_->publish(grid);
        }

        void publish_grid(Transformation3D twc)
        {
            std::vector<Point3D> list;

            const Eigen::Vector3d reference(twc.getPosition().x(), twc.getPosition().y(), twc.getPosition().z());

            const Eigen::Vector3d radius_vec{10.0f, 10.0f, 0.0f};
            const Eigen::Vector3d lower_bound{reference - radius_vec};
            const Eigen::Vector3d upper_bound{reference + radius_vec};

            const wavemap::Point3D lower_bound_wavemap = lower_bound.cast<float>();
            const wavemap::Point3D upper_bound_wavemap = upper_bound.cast<float>();

            if (const auto *hashed_wavelet_octree =
                    dynamic_cast<const HashedWaveletOctree *>(_map.get());
                hashed_wavelet_octree)
            {

                const wavemap::FloatingPoint min_cell_width = hashed_wavelet_octree->getMinCellWidth();
                const wavemap::Index3D min_corner_index =
                    wavemap::convert::pointToFloorIndex(lower_bound_wavemap,
                                                        1.f / min_cell_width);
                const wavemap::Index3D max_corner_index =
                    wavemap::convert::pointToCeilIndex(upper_bound_wavemap,
                                                       1.f / min_cell_width);

                wavemap::QueryAccelerator query_accelerator(*hashed_wavelet_octree);
                for (const auto &query_index :
                     wavemap::Grid<3>(min_corner_index, max_corner_index))
                {
                    const wavemap::FloatingPoint occupancy_log_odds = query_accelerator.getCellValue(query_index);
                    if (occupancy_log_odds > 0.5)
                    {
                        const auto index = OctreeIndex{0, query_index};
                        const Point3D block = convert::indexToCenterPoint(index.position, min_cell_width);

                        list.emplace_back(block);
                    }
                }
            }

            if (const auto *hashed_chunked_wavelet_octree =
                    dynamic_cast<const HashedWaveletOctree *>(_map.get());
                hashed_chunked_wavelet_octree)
            {

                const wavemap::FloatingPoint min_cell_width = hashed_chunked_wavelet_octree->getMinCellWidth();
                const wavemap::Index3D min_corner_index =
                    wavemap::convert::pointToFloorIndex(lower_bound_wavemap,
                                                        1.f / min_cell_width);
                const wavemap::Index3D max_corner_index =
                    wavemap::convert::pointToCeilIndex(upper_bound_wavemap,
                                                       1.f / min_cell_width);

                wavemap::QueryAccelerator query_accelerator(*hashed_chunked_wavelet_octree);
                for (const auto &query_index :
                     wavemap::Grid<3>(min_corner_index, max_corner_index))
                {
                    const wavemap::FloatingPoint occupancy_log_odds = query_accelerator.getCellValue(query_index);
                    if (occupancy_log_odds > 0.5)
                    {
                        const auto index = OctreeIndex{0, query_index};
                        const Point3D block = convert::indexToCenterPoint(index.position, min_cell_width);

                        list.emplace_back(block);
                    }
                }
            }
 
            transfer_grid(&list, reference.cast<float>()); 
        }

        uint64_t encode(Point3D p, float extension, float resolution)
        {
            uint16_t f = static_cast<uint16_t>( extension / resolution );
            return (static_cast<uint64_t>( (p.x() + f ) * f * 2) & 0x1FFFFFUL ) |
                   (static_cast<uint64_t>( (p.y() + f ) * f * 2) & 0x1FFFFFUL ) << 21 |
                   (static_cast<uint64_t>( (p.z() + f ) * f * 2) & 0x1FFFFFUL ) << 42;
        }

        rclcpp::Node *_node;
        rclcpp::Publisher<msp_msgs::msg::MicroGrid>::SharedPtr microgrid_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        Timestamp last_run_timestamp_internal_;

        wavemap::MapBase::Ptr _map;
        std::shared_ptr<TfTransformer> transformer_;
    };
}
