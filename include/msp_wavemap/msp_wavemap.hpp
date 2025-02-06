#pragma once
#include <msp_controller/msp_node_base.hpp>
#include <msp_controller/msp_px4.h>
#include <msp_msgs/srv/trajectory_check.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <msp_msgs/msg/trajectory.hpp>
#include <msp_wavemap/ros2/ros2map_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <wavemap/core/config/config_base.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/pipeline.h>

using namespace wavemap;

namespace msp
{

  class MSWaveMapNode : public msp::MSPNodeBase // public rclcpp::Node
  {
  public:
    explicit MSWaveMapNode() : msp::MSPNodeBase("MSWaveMapNode", MSP_COMP_AVOID)
    {

      cam2body_ << 0.0, 0.0, 1.0, 0.0,
          -1.0, 0.0, 0.0, 0.0,
          0.0, -1.0, 0.0, -0.02,
          0.0, 0.0, 0.0, 1.0;

      gz_node = std::make_unique<gz::transport::Node>();

      msp_trajectory_check = this->create_service<msp_msgs::srv::TrajectoryCheck>(
          "/msp/in/trajectory_check",
          std::bind(&MSWaveMapNode::onTrajectoryCheck, this, std::placeholders::_1, std::placeholders::_2));

      attitude_subscription = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
          "/msp/out/vehicle_attitude", this->getQos(), [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg)
          {
            Eigen::Quaternion<float> quaternion(msg->q[0], // w
                                                -msg->q[1], // x
                                                -msg->q[2], // y
                                                 msg->q[3]  // z
            );
            //  quaternion.normalize();
            Eigen::Matrix3f body_r_m = quaternion.toRotationMatrix();
            Eigen::Matrix4f body2world;
            body2world.block<3, 3>(0, 0) = body_r_m;
            body2world(0, 3) = pos_[0];
            body2world(1, 3) = pos_[1];
            body2world(2, 3) = pos_[2];
            body2world(3, 3) = 1.0;

            Eigen::Matrix4f cam_T = body2world * cam2body_;
            Eigen::Quaternionf q(cam_T.block<3, 3>(0, 0));

            T_W_C.getRotation() = Rotation3D(q);
            T_W_C.getPosition() << cam_T(0, 3), cam_T(1, 3), cam_T(2, 3); });

      local_pos_subscription = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/msp/out/vehicle_local_position", this->getQos(), [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
          { pos_ << msg->x, msg->y, -msg->z; });

      initialize();
    }

    void initialize();

  private:
    std::unique_ptr<gz::transport::Node> gz_node;

    // Map data structure
    MapBase::Ptr occupancy_map_;

    // Threadpool shared among all input handlers and operations
    std::shared_ptr<ThreadPool> thread_pool_;

    // Map management pipeline
    std::shared_ptr<Pipeline> pipeline_;

    Transformation3D T_W_C{};

    Eigen::Vector3f pos_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Matrix4f cam2body_;

    cv::Mat image;

    int count;

    msp::MSPMap2ROS2Publisher map_publisher = msp::MSPMap2ROS2Publisher(this);

    rclcpp::Service<msp_msgs::srv::TrajectoryCheck>::SharedPtr msp_trajectory_check;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_subscription;

    rclcpp::Publisher<msp_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

    void onDepthReceived(const gz::msgs::Image &msg);
    void onTrajectoryCheck(const std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Request> request,
                           std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Response> response);
  };

} // namespace msp
