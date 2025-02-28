#pragma once
#include <msp_wavemap/lib/sdf/esdf_generator.hpp>
#include <msp_wavemap/ros2/ros2_esdf_publisher.hpp>

using namespace wavemap;

namespace msp
{
    class MSPESDFGenerator
    {
    public:
        explicit MSPESDFGenerator(rclcpp::Node *node, std::shared_ptr<TfTransformer> transformer) : node_(node), 
        transformer_(transformer)
        {
        }

        void setMap(MapBase::Ptr map, std::shared_ptr<ThreadPool> thread_pool = nullptr) {

            RCLCPP_INFO(node_->get_logger(), "MSP WaveMap Node started");
            esdf_generator_ = new ESDFGenerator(map, dimensions_, thread_pool);
            esdf_publisher_ = new MSPRos2ESDFPublisher(node_,transformer_,esdf_generator_); 
        }

        void generate(const Transformation3D& T_W_C) {

            const Point3D reference = T_W_C.getPosition();
            esdf_generator_->generate_fast_sweep(reference);
        }

        std::shared_ptr<msp::ESDF> get() { return esdf_generator_->getESDF(); }


    private:

        const Point3D dimensions_ = Point3D(10.0f, 10.0f, 5.f);
        
        rclcpp::Node* node_;
        msp::MSPRos2ESDFPublisher* esdf_publisher_;
        msp::ESDFGenerator* esdf_generator_;

        std::shared_ptr<TfTransformer> transformer_;
    };
}