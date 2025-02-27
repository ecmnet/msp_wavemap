#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <msp_msgs/msg/micro_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

    using namespace wavemap;

    class MSPRos2PCLPublisher
    {

    public:
        explicit MSPRos2PCLPublisher(rclcpp::Node *node, std::shared_ptr<TfTransformer> transformer) : _node(node), transformer_(transformer)
        {

            point_cloud_publisher_ = _node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);

        }

        void setMap(wavemap::MapBase::Ptr map)
        {
            _map = map;
            timer_ = _node->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(&MSPRos2PCLPublisher::publish_point_cloud, this));
        }

    private:

        void publish_point_cloud()
        {

            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            std::vector<Point3D> list;

            // Define a functor that converts map leaf nodes into pointcloud points
            auto add_points_for_leaf_node =
                [this, min_cell_width = _map->getMinCellWidth(),
                 occupancy_threshold = 0.5f,
                 pcl = &pcl_cloud](
                    const OctreeIndex &node_index, FloatingPoint node_log_odds)
            {
                if (node_log_odds > occupancy_threshold)
                {

                    const FloatingPoint cell_width =
                        convert::heightToCellWidth(min_cell_width, node_index.height);

                    const Point3D block = convert::indexToCenterPoint(
                        node_index.position, cell_width);

                    const float min_width_2 = min_cell_width / 2.0f;
                    const float cell_width_2 = cell_width / 2.0f;

                    for (float sx = -cell_width_2; sx < cell_width_2; sx += min_cell_width)
                        for (float sy = -cell_width_2; sy < cell_width_2; sy += min_cell_width)
                            for (float sz = -cell_width_2; sz < cell_width_2; sz += min_cell_width)
                            {
                                const pcl::PointXYZ pcl_point(block.x() + sx + min_width_2,
                                                              block.y() + sy + min_width_2,
                                                              block.z() + sz + min_width_2);
                                pcl->emplace_back(pcl_point);
                            }
                }
            };

            // Define a functor that allows us to skip unchanged map blocks
            auto process_block_if_changed = [this, &add_points_for_leaf_node](
                                                const Index3D &block_index,
                                                const auto &block)
            {
                const bool block_changed =
                    last_run_timestamp_internal_ < block.getLastUpdatedStamp();
                if (block_changed)
                {
                    block.forEachLeaf(block_index, add_points_for_leaf_node);
                }
            };

            if (const auto *hashed_wavelet_octree =
                    dynamic_cast<const HashedWaveletOctree *>(_map.get());
                hashed_wavelet_octree)
            {
                hashed_wavelet_octree->forEachBlock(process_block_if_changed);
            }
            else if (const auto *hashed_chunked_wavelet_octree =
                         dynamic_cast<const HashedChunkedWaveletOctree *>(
                             _map.get());
                     hashed_chunked_wavelet_octree)
            {
                hashed_chunked_wavelet_octree->forEachBlock(process_block_if_changed);
            }
            else
            {
                // Fallback for non-hashed map types: simply process all leaves
                _map->forEachLeaf(add_points_for_leaf_node);
            }

            last_run_timestamp_internal_ = Time::now();

            if (pcl_cloud.points.size() > 0)
            {
                sensor_msgs::msg::PointCloud2 ros_cloud;
                pcl_cloud.width = pcl_cloud.points.size();
                pcl_cloud.height = 1;
                pcl::toROSMsg(pcl_cloud, ros_cloud);
                ros_cloud.header.frame_id = "world";
                ros_cloud.header.stamp = _node->get_clock()->now();
                point_cloud_publisher_->publish(ros_cloud);
            }
        }


        rclcpp::Node *_node;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        Timestamp last_run_timestamp_internal_;

        wavemap::MapBase::Ptr _map;
        std::shared_ptr<TfTransformer> transformer_;
    };
}
