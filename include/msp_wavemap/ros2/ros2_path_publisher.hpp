
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace msp
{

    using namespace wavemap;

    class MSPRos2PathPublisher
    {
        public:
        explicit MSPRos2PathPublisher(rclcpp::Node *node) : node_(node) {
            path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("path", 10);
        }

         void push(Point3D p) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = node_->get_clock()->now();
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();
            poses_.push_back(pose);
        }

         void publish() {
            nav_msgs::msg::Path path;
            path.header.frame_id = "world";
            path.header.stamp = node_->get_clock()->now();
            path.poses = poses_;
            path_publisher_->publish(path);
            poses_.clear();
        }


        private:

        rclcpp::Node *node_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    
        std::vector<geometry_msgs::msg::PoseStamped> poses_;


    };
}
