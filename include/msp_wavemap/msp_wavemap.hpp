#pragma once
#include <msp_controller/msp_node_base.hpp>
#include <msp_controller/msp_px4.h>
#include <msp_msgs/srv/trajectory_check.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
// #include <msp_msgs/msg/trajectory.hpp>
#include <msp_wavemap/ros2/ros2map_publisher.hpp>
#include <msp_wavemap/ros2/msp_grid_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <msp_wavemap/msp_waverider.hpp>

#include "msp_controller/ros2/tf_transformer.h"

#include <wavemap/core/config/config_base.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/pipeline.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace wavemap;

namespace msp
{

  class MSWaveMapNode : public msp::MSPNodeBase // public rclcpp::Node
  {
  public:
    explicit MSWaveMapNode() : msp::MSPNodeBase("MSWaveMapNode", MSP_COMP_MAP)
    {

      gz_node = std::make_unique<gz::transport::Node>();

      // msp_trajectory_check = this->create_service<msp_msgs::srv::TrajectoryCheck>(
      //     "/msp/in/trajectory_check",
      //     std::bind(&MSWaveMapNode::onTrajectoryCheck, this, std::placeholders::_1, std::placeholders::_2));

      initialize();
    }

    void initialize();

  protected:
    void onArmingState(uint8_t arming_state ) override
    {
      if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED )
      {
        RCLCPP_INFO(this->get_logger(), "Clearing map");
        occupancy_map_->clear();
      }
    }

  private:
    std::unique_ptr<gz::transport::Node> gz_node;

    // Map data structure
    MapBase::Ptr occupancy_map_;

    // Threadpool shared among all input handlers and operations
    std::shared_ptr<ThreadPool> thread_pool_;

    // Map management pipeline
    std::shared_ptr<Pipeline> pipeline_;

    Eigen::Vector3f pos_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    cv::Mat image;

    int count;
    bool in_collision = false;

    msp::MSPWaveRider wave_rider_ = msp::MSPWaveRider(this, "world", transformer_);

    msp::MSPMap2ROS2Publisher pcl_publisher = msp::MSPMap2ROS2Publisher(this, transformer_);
    msp::MSPGridPublisher msp_publisher = msp::MSPGridPublisher(this, transformer_);

    rclcpp::Service<msp_msgs::srv::TrajectoryCheck>::SharedPtr msp_trajectory_check;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    // rclcpp::Publisher<msp_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

    void onDepthReceived(const gz::msgs::Image &msg);
    void onTrajectoryCheck(const std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Request> request,
                           std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Response> response);

    void publish_camera_transform()
    {

      static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      geometry_msgs::msg::TransformStamped transformStamped;

      transformStamped.header.stamp = this->get_clock()->now();
      transformStamped.header.frame_id = "base_link";  // Parent frame
      transformStamped.child_frame_id = "camera_link"; // Child frame

      // Translation (Camera position relative to body frame)
      transformStamped.transform.translation.x = 0.14; // 10 cm forward
      transformStamped.transform.translation.y = 0.00; // 5 cm right
      transformStamped.transform.translation.z = 0.0;  // 20 cm up

      tf2::Quaternion q;
      q.setRPY(-M_PI_2, 0, -M_PI_2);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      static_broadcaster_->sendTransform(transformStamped);
    }
  };

} // namespace msp
