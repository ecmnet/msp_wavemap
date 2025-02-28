#pragma once
#include <msp_wavemap/lib/sdf/esdf_generator.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace wavemap;

namespace msp
{
    class MSPRos2ESDFPublisher
    {

    public:
        explicit MSPRos2ESDFPublisher(rclcpp::Node *node, std::shared_ptr<TfTransformer> transformer, ESDFGenerator* esdf) 
        : node_(node), transformer_(transformer), esdf_(esdf)
        {
            esdf_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("esdf", 1);
            timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MSPRos2ESDFPublisher::publish, this));
        }

    private:

        void publish()
        {
            const auto T_W_C = transformer_->lookupLatestTransform("world", "camera_link");

            if (T_W_C == std::nullopt)
                return;

            publish_esdf(*T_W_C);
        }

        void publish_esdf(Transformation3D twc)
        {

            const auto esdf = esdf_->getESDF();
            const auto esdf_data = esdf->getData();

            if( esdf_data->empty())
              return;
              
            const Point3D reference(twc.getPosition().x(), twc.getPosition().y(), twc.getPosition().z());
            const uint32_t offset = esdf->getOffsetZ() * esdf->getSizeY() * esdf->getSizeX();

            visualization_msgs::msg::MarkerArray esdf_marker_array;
            int count = 0;

            for (uint32_t y = 0; y < esdf->getSizeY(); y++)
            {
                for (uint32_t x = 0; x < esdf->getSizeX(); x++)
                {
                    uint32_t index = offset + y * esdf->getSizeX() + x;
                    const float v = esdf_data->at(index);
                    if (v > MAX_DISTANCE)
                        continue;

                    const Point3D p = esdf->index_to_World(index, reference);
                    //  std::cout << "ESDF: " << p.transpose() << " " << v << std::endl;
                    esdf_marker_array.markers.push_back(buildMarker(p, v, count++));
                }
            }

            esdf_publisher_->publish(esdf_marker_array);
        }

        visualization_msgs::msg::Marker buildMarker(Point3D position, float v, int i)
        {

            const float norm = std::clamp(v / 15.0f, 0.0f, 1.0f);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.id = i;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.01;
            marker.color.r = std::min(3.0f * norm, 1.0f);
            marker.color.g = std::min(3.0f * norm - 1.0f, 1.0f);
            marker.color.b = std::min(3.0f * norm - 2.0f, 1.0f);
            marker.color.a = 0.7f;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::MODIFY;
            return marker;
        }

        msp::ESDFGenerator* esdf_;
        rclcpp::Node* node_;
        std::shared_ptr<TfTransformer> transformer_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr esdf_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}