#pragma once
#include <msp_wavemap/lib/sdf/esdf_generator.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"

#define MAX_DISTANCE 99.0f

using namespace wavemap;

namespace msp
{
    class MSPESDFGenerator
    {
    public:
        explicit MSPESDFGenerator(rclcpp::Node *node, std::shared_ptr<TfTransformer> transformer) : _node(node), 
        transformer_(transformer)
        {
            grid_publisher_ = _node->create_publisher<visualization_msgs::msg::MarkerArray>("esdf", 100);
        }

        void setMap(MapBase::Ptr map, std::shared_ptr<ThreadPool> thread_pool = nullptr) {

            std::cout << "Creating ESDF generator" << std::endl;
            esdf_generator = new ESDFGenerator(map, dimensions_, thread_pool);

            timer_ = _node->create_wall_timer(
                std::chrono::milliseconds(250),
                std::bind(&MSPESDFGenerator::publish, this));
                
           
        }

        void generate(const Transformation3D& T_W_C) {

            const Point3D reference = T_W_C.getPosition();

            auto t1 = std::chrono::high_resolution_clock::now();
            esdf_generator->generate_fast_sweep(reference);
            auto t2 = std::chrono::high_resolution_clock::now();
  
            std::cout << "ESDF: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000 << "ms\n";
        }

        void publish()
        {
            const auto T_W_C = transformer_->lookupLatestTransform("world", "camera_link");

            if (T_W_C == std::nullopt)
                return;

            publish_esdf(*T_W_C);
        }

        void publish_esdf(Transformation3D twc)
        {

            const auto esdf_data = esdf_generator->getData();
            const auto esdf_prop = esdf_generator->getProperties();

            const Point3D reference(twc.getPosition().x(), twc.getPosition().y(), twc.getPosition().z());

            const uint32_t offset = esdf_prop->getOffsetZ()* esdf_prop->getSizeY() * esdf_prop->getSizeX()  ;
            
            visualization_msgs::msg::MarkerArray esdf;
            int count = 0;

            for(uint32_t y = 0; y < esdf_prop->getSizeY(); y++) {
                for(uint32_t x = 0; x < esdf_prop->getSizeX(); x++) {
                    uint32_t index = offset + y*esdf_prop->getSizeX() + x;
                    const float v = esdf_data->at(index);
                     if(v > MAX_DISTANCE)
                        continue;
                    
                    const Point3D p = esdf_prop->index_to_World(index,reference);
                   //  std::cout << "ESDF: " << p.transpose() << " " << v << std::endl;
                   esdf.markers.push_back(buildMarker(p,v,count++));
                }
            }

            grid_publisher_->publish(esdf);
        }

        visualization_msgs::msg::Marker buildMarker(Point3D position, float v, int i) {

           const float norm = std::clamp(v/15.0f,0.0f,1.0f);

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
    

    private:

        const Point3D dimensions_ = Point3D(10.0f, 10.0f, 5.f);
        
        rclcpp::Node *_node;
        std::shared_ptr<TfTransformer> transformer_;
        msp::ESDFGenerator * esdf_generator;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
       
    };
}