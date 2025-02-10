#ifndef WAVEMAP_ROS_UTILS_TF_TRANSFORMER_H_
#define WAVEMAP_ROS_UTILS_TF_TRANSFORMER_H_

#include <map>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>  
#include <tf2_ros/transform_listener.h>
#include <wavemap/core/common.h>

namespace wavemap {
class TfTransformer {
 public:
  explicit TfTransformer(FloatingPoint tf_buffer_cache_time = 10.0f)
   : clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),  
     tf_buffer_(clock_),  
     tf_listener_(tf_buffer_) {}

  // Check whether a transform is available
  bool isTransformAvailable(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time&  frame_timestamp) const;

  // Waits for a transform to become available, while doing less aggressive
  // polling that ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string& to_frame_id,
                        const std::string& from_frame_id,
                        const rclcpp::Time&  frame_timestamp);

  // Lookup transforms and convert them to Kindr
  std::optional<Transformation3D> lookupTransform(
      const std::string& to_frame_id, const std::string& from_frame_id,
      const rclcpp::Time&  frame_timestamp);
  std::optional<Transformation3D> lookupLatestTransform(
      const std::string& to_frame_id, const std::string& from_frame_id);

  // Strip leading slashes if needed to avoid TF errors
  static std::string sanitizeFrameId(const std::string& string);

 private:
 rclcpp::Clock::SharedPtr clock_; 
 tf2_ros::Buffer tf_buffer_;
 tf2_ros::TransformListener tf_listener_;

  // Transform lookup timers
  // Timeout between each update attempt
  const rclcpp::Duration transform_lookup_retry_period_ = rclcpp::Duration(std::chrono::milliseconds(020));
  // Maximum time to wait before giving up
  const rclcpp::Duration transform_lookup_max_time_ =  rclcpp::Duration(std::chrono::milliseconds(250));

  bool waitForTransformImpl(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time&  frame_timestamp) const;
  std::optional<Transformation3D> lookupTransformImpl(
      const std::string& to_frame_id, const std::string& from_frame_id,
      const rclcpp::Time&  frame_timestamp);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_UTILS_TF_TRANSFORMER_H_
