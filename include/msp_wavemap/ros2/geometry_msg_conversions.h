#ifndef WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_

//#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/point32.h>
#include "geometry_msgs/msg/point.hpp"


namespace wavemap::convert {
inline Point3D pointMsgToPoint3D(const geometry_msgs::msg::Point& msg) {
  Point3D point;
  point.x() = msg.x;
  point.y() = msg.y;
  point.z() = msg.z;
  return  point;
}

inline geometry_msgs::msg::Point point3DToPointMsg(const Point3D& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

// inline Point3D point32MsgToPoint3D(const geometry_msgs::msg::Point32& msg) {
//   return {msg.x, msg.y, msg.z};
// }

// inline geometry_msgs::msg::Point32 point3DToPoint32Msg(const Point3D& point) {
//   geometry_msgs::msg::Point32 msg;
//   msg.x = point.x();
//   msg.y = point.y();
//   msg.z = point.z();
//   return msg;
// }

inline Vector3D vector3MsgToVector3D(const geometry_msgs::msg::Vector3& msg) {
  Vector3D point;
  point.x() = msg.x;
  point.y() = msg.y;
  point.z() = msg.z;
  return  point;
}

inline geometry_msgs::msg::Vector3 vector3DToVector3Msg(const Vector3D& vector) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
  return msg;
}

inline Rotation3D quaternionMsgToRotation3D(
    const geometry_msgs::msg::Quaternion& msg) {
  Eigen::Quaterniond rotation_double;
  rotation_double.w() = msg.w;
  rotation_double.x() = msg.x;
  rotation_double.y() = msg.y;
  rotation_double.z() = msg.z;
  return Rotation3D{rotation_double.cast<FloatingPoint>()};
}

inline Transformation3D transformMsgToTransformation3D(
    const geometry_msgs::msg::Transform& msg) {
  return Transformation3D{quaternionMsgToRotation3D(msg.rotation),
                          vector3MsgToVector3D(msg.translation)};
                      
}

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
