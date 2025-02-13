
#pragma once
// Map
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

// Policies
#include <rmpcpp/policies/simple_target_policy.h>
#include <waverider/goal_policy.h>
#include <waverider/goal_policy_tuning.h>
#include <waverider/obstacle_list_policy.h>
#include <waverider/waverider_policy.h>
#include <waverider/yaw_policy.h>
#include <waverider/yaw_policy_tuning.h>

// Config
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/core/config/value_with_unit.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// MSP Controller
#include "msp_controller/ros2/tf_transformer.h"

using namespace wavemap;
using namespace waverider;

namespace msp
{

    struct MSPWaveRiderConfig
        : wavemap::ConfigBase<MSPWaveRiderConfig, 13, StringList,
                              GoalPolicyTuning, YawPolicyTuning,
                              ObstaclePolicyTuning>
    {
        std::string odom_frame = "camera_frame";

        std::string goal_tf_frame;
        std::string ground_plane_tf_frame;
        ValueWithUnit<SiUnit::kSeconds, FloatingPoint> tf_lookup_delay = 0.05f;
        ValueWithUnit<SiUnit::kMeters, FloatingPoint> ground_plane_offset = 0.f;

        FloatingPoint occupancy_threshold = 0.1f;

        ValueWithUnit<SiUnit::kSeconds, FloatingPoint> control_period = 0.02f;
        ValueWithUnit<SiUnit::kSeconds, FloatingPoint> integrator_step_size = 0.005f;

        int publish_debug_visuals_every_n_iterations = 20;

        GoalPolicyTuning goal_policy;
        YawPolicyTuning yaw_policy;
        ObstaclePolicyTuning map_obstacles_policy;
        ObstaclePolicyTuning aabb_obstacles_policy;
        // FloatingPoint goal_policy_marker_scale = 0.1f;
        // FloatingPoint yaw_policy_marker_scale = 0.1f;
        // FloatingPoint map_obstacles_policy_marker_scale = 0.1f;
        // FloatingPoint aabb_obstacles_policy_marker_scale = 0.1f;

        static MemberMap memberMap;

        bool isValid(bool verbose) const override;
    };

    class MSPWaveRider
    {
    public:
        MSPWaveRider(rclcpp::Node* node, std::string map_frame, std::shared_ptr<TfTransformer> transformer);

        void updateMap(const wavemap::MapBase::Ptr  map);

    private:
        

        MSPWaveRiderConfig readConfig();
        std::optional<Plane3D> getGroundPlaneFromTf();

        MSPWaveRiderConfig config_;
        std::string map_frame_;

        GoalPolicy goal_policy_;
        YawPolicy yaw_policy_;
        WaveriderPolicy map_obstacles_policy_;
        ObstacleListPolicy aabb_obstacles_policy_;

        std::shared_ptr<TfTransformer> transformer_;
        

        rclcpp::Node* node_;
    };

}
