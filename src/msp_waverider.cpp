#include <msp_wavemap/msp_waverider.hpp>
#include <msp_wavemap/lib/config/stream_conversions.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <waverider/goal_policy_tuning.h>

using namespace wavemap;
using namespace waverider;

namespace msp
{
    DECLARE_CONFIG_MEMBERS(MSPWaveRiderConfig,
                           (odom_frame)(goal_tf_frame)(ground_plane_tf_frame)(tf_lookup_delay)(ground_plane_offset)(occupancy_threshold)(control_period)(integrator_step_size)(publish_debug_visuals_every_n_iterations)(goal_policy)(yaw_policy)(map_obstacles_policy)(aabb_obstacles_policy));

    bool MSPWaveRiderConfig::isValid(bool verbose) const
    {
        bool all_valid = true;

        all_valid &= IS_PARAM_NE(odom_frame, "", verbose);
        all_valid &= IS_PARAM_NE(goal_tf_frame, "", verbose);
        all_valid &= IS_PARAM_NE(ground_plane_tf_frame, "", verbose);
        all_valid &= IS_PARAM_GE(tf_lookup_delay, 0.f, verbose);
        all_valid &= IS_PARAM_GT(control_period, 0.f, verbose);
        all_valid &= IS_PARAM_TRUE(goal_policy.isValid(verbose), verbose);
        all_valid &= IS_PARAM_TRUE(yaw_policy.isValid(verbose), verbose);
        all_valid &= IS_PARAM_TRUE(map_obstacles_policy.isValid(verbose), verbose);
        all_valid &= IS_PARAM_TRUE(aabb_obstacles_policy.isValid(verbose), verbose);

        return all_valid;
    }

    MSPWaveRider::MSPWaveRider(rclcpp::Node *node, std::string map_frame, std::shared_ptr<TfTransformer> transformer) : node_(node), map_frame_(map_frame), transformer_(transformer), config_(readConfig())
    {
        CHECK(config_.isValid(true));

        // Configure policies
        goal_policy_.setTuning(config_.goal_policy);
        yaw_policy_.setTuning(config_.yaw_policy);
        map_obstacles_policy_.setOccupancyThreshold(config_.occupancy_threshold);
        map_obstacles_policy_.setTuning(config_.map_obstacles_policy);
        aabb_obstacles_policy_.setTuning(config_.aabb_obstacles_policy);

        RCLCPP_INFO(node_->get_logger(), "MSP WaveRider started successfully");
    }

    void MSPWaveRider::updateMap(const wavemap::MapBase::Ptr map)
    {

        const auto ground_plane = getGroundPlaneFromTf();
        if (!ground_plane.has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "Ground plane TF lookup failed. Could not extract obstacles.");
            return;
        }

        const auto T_W_C = transformer_->lookupLatestTransform("base_link", "world");


        if (auto* hashed_map = dynamic_cast<HashedWaveletOctree*>(map.get());
               hashed_map) {
            map_obstacles_policy_.updateObstacles(*hashed_map, T_W_C->getPosition(),
                                                  *ground_plane);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                "Waverider policies can currently only be extracted from maps of "
                "type wavemap::HashedWaveletOctree.");
        }
    }

    MSPWaveRiderConfig MSPWaveRider::readConfig()
    {

        std::string base_path = ament_index_cpp::get_package_share_directory("msp_wavemap");
        std::string config_name = base_path + "/config/alma.yaml";

        std::ifstream file(config_name);
        if (!file.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not open waverider config: %s", config_name.c_str());
            return MSPWaveRiderConfig();
        }

        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        std::istringstream iss(content);

        const auto params = wavemap::io::yamlStreamToParams(iss);

        auto config = MSPWaveRiderConfig::from(params.value());
        if (config)
            return config.value();

        return MSPWaveRiderConfig();
    }

    std::optional<Plane3D> MSPWaveRider::getGroundPlaneFromTf()
    {
        auto T_W_G = transformer_->lookupLatestTransform(map_frame_, config_.ground_plane_tf_frame);

        if (T_W_G != std::nullopt)
        {
            Plane3D ground_plane;
            ground_plane.normal = T_W_G->getRotation().rotate(Vector3D::UnitZ());
            ground_plane.offset = ground_plane.normal.dot(T_W_G->getPosition()) +
                                  config_.ground_plane_offset;
            return ground_plane;
        }
        return std::nullopt;
    }

}