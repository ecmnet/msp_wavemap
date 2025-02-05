#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/indexing/index_conversions.h>

#include <algorithm>
#include <memory>

// TODO: Skip unchanged block as described in 
   https://github.com/ethz-asl/wavemap/blob/main/interfaces/ros1/wavemap_ros/src/map_operations/publish_pointcloud_operation.cc

namespace msp
{

    using namespace wavemap;

    class MSPMap2ROS2Publisher
    {

    public:
        explicit MSPMap2ROS2Publisher(rclcpp::Node *node) : _node(node)
        {

            point_cloud_publisher_ = _node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
            timer_ = _node->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&MSPMap2ROS2Publisher::publish_point_cloud, this));
        }

        void setMap(wavemap::MapBase::Ptr map)
        {
            _map = map;
        }

    private:
        void publish_point_cloud()
        {

            if (!_map)
                return;

            // std::shared_ptr<wavemap::HashedChunkedWaveletOctree> _omap =
            //     std::dynamic_pointer_cast<wavemap::HashedChunkedWaveletOctree>(_map);

            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            _map->forEachLeaf([&](const wavemap::OctreeIndex &index, float occupancy)
                               {      
            if (occupancy > 0.5f) { 
                   Point<3> world_point = wavemap::convert::nodeIndexToCenterPoint(index,0.1f);
                   pcl_cloud.push_back(pcl::PointXYZ(world_point.x(), world_point.y(), world_point.z()));
             } });
            pcl_cloud.width = pcl_cloud.points.size();
            pcl_cloud.height = 1;
            pcl::toROSMsg(pcl_cloud, ros_cloud);
            ros_cloud.header.frame_id = "world";
            ros_cloud.header.stamp = _node->get_clock()->now();
            point_cloud_publisher_->publish(ros_cloud);
        }

        rclcpp::Node *_node;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        wavemap::MapBase::Ptr _map;
    };

}
