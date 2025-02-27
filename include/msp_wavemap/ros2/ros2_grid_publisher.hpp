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


using namespace wavemap;

namespace msp
{
    class MSPRos2GridPublisher
       
    {
        using BlockIndex = Index3D;
        using CellIndex = OctreeIndex;

    public:
        explicit MSPRos2GridPublisher(rclcpp::Node *node, std::shared_ptr<TfTransformer> transformer) : _node(node), transformer_(transformer)
        {

            microgrid_publisher_ = _node->create_publisher<msp_msgs::msg::MicroGrid>("msp/in/micro_grid", 1);
        }

        void setMap(wavemap::MapBase::Ptr map)
        {
            _map = map;
            timer_ = _node->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MSPRos2GridPublisher::publish, this));
        }

    private:
        void publish()
        {
            const auto T_W_C = transformer_->lookupLatestTransform("world", "camera_link");

            if (!_map || T_W_C == std::nullopt)
                return;

            publish_grid(*T_W_C);
        }

        void transfer_grid(std::vector<uint64_t> *list, Point3D r)
        {
            msp_msgs::msg::MicroGrid grid;

            if (list->size() == 0)
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
                grid.data[grid.count] = list->back();
                list->pop_back();
                if (grid.count >= 25)
                {
                    count++;
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
            std::vector<uint64_t> add, remove;

            const Eigen::Vector3d reference(twc.getPosition().x(), twc.getPosition().y(), twc.getPosition().z());

            const Eigen::Vector3d radius_vec{10.0f, 10.0f, 0.1f};
            const Eigen::Vector3d lower_bound{reference - radius_vec};
            const Eigen::Vector3d upper_bound{reference + radius_vec};

            const wavemap::Point3D lower_bound_wavemap = lower_bound.cast<float>();
            const wavemap::Point3D upper_bound_wavemap = upper_bound.cast<float>();

            if(!_map || _map->empty())
              return;

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
                   
                    const BlockIndex block_index = convert::indexToBlockIndex(query_index, hashed_wavelet_octree->getTreeHeight());
                    const auto *b = hashed_wavelet_octree->getBlock(block_index);
                    if (!b || b->getTimeSinceLastUpdated() > 0.2f)
                        continue;
             
                    const wavemap::FloatingPoint occupancy_log_odds = query_accelerator.getCellValue(query_index);

                    const auto index = OctreeIndex{0, query_index};
                    const Point3D block = convert::indexToCenterPoint(index.position, min_cell_width);
                    if (occupancy_log_odds > 0.5)
                        add.emplace_back(encode(block, radius_vec.x(), min_cell_width, 0.7f));
                    if (occupancy_log_odds < -1.0)
                        remove.emplace_back(encode(block, radius_vec.x(), min_cell_width, 0));
                   
                }
            }

            if ( const auto *hashed_chunked_wavelet_octree =
                    dynamic_cast< const HashedChunkedWaveletOctree *>(_map.get());
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

                    const BlockIndex block_index = convert::indexToBlockIndex(query_index, hashed_chunked_wavelet_octree->getTreeHeight());
                    const auto *b = hashed_chunked_wavelet_octree->getBlock(block_index);
                    if (!b || b->getTimeSinceLastUpdated() > 0.2f)
                        continue;
                   
                    const wavemap::FloatingPoint occupancy_log_odds = query_accelerator.getCellValue(query_index);
                    
                    const auto index = OctreeIndex{0, query_index};
                    const Point3D block = convert::indexToCenterPoint(index.position, min_cell_width);

                    if (occupancy_log_odds > 0.5)
                        add.emplace_back(encode(block, radius_vec.x(), min_cell_width, 0.7f));
                    if (occupancy_log_odds < -1.0)
                        remove.emplace_back(encode(block, radius_vec.x(), min_cell_width, 0));

                }
            }

            transfer_grid(&remove, reference.cast<float>());
            transfer_grid(&add, reference.cast<float>());
        }

        uint64_t encode(Point3D p, float extension, float resolution, float value = 0)
        {
            uint16_t f = static_cast<uint16_t>(extension / resolution);
            return (static_cast<uint64_t>((value) * 4096.0f) & 0xFFFUL) |
                   (static_cast<uint64_t>((p.x() + f) * f * 2) & 0x1FFFFUL) << 12 |
                   (static_cast<uint64_t>((p.y() + f) * f * 2) & 0x1FFFFUL) << 29 |
                   (static_cast<uint64_t>((p.z() + f) * f * 2) & 0x1FFFFUL) << 46;
        }

        rclcpp::Node *_node;
        rclcpp::Publisher<msp_msgs::msg::MicroGrid>::SharedPtr microgrid_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        Timestamp last_run_timestamp_internal_;

        wavemap::MapBase::Ptr _map;
        std::shared_ptr<TfTransformer> transformer_;
    };
}
