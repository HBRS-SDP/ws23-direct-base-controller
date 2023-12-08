#include "component_wise_pose_error_calculator.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
bool get_component_wise_pose_error(const geometry_msgs::msg::PoseStamped::SharedPtr origin_pose,
                                   const geometry_msgs::msg::PoseStamped::SharedPtr target_pose,
                                   ComponentWiseCartesianDifference &error,
                                   std::unique_ptr<tf2_ros::Buffer> &tf_buffer_)
{
  if (!origin_pose || !target_pose)
  {
    return false; /* code */
  }
  geometry_msgs::msg::PoseStamped pose_out_;
  transform_pose(origin_pose, target_pose, tf_buffer_, pose_out_);
  error.linear_x = target_pose->pose.position.x - origin_pose->pose.position.x;
  error.linear_y = target_pose->pose.position.y - origin_pose->pose.position.y;
  error.linear_z = target_pose->pose.position.z - origin_pose->pose.position.z;

  tf2::Quaternion q1(origin_pose->pose.orientation.x, origin_pose->pose.orientation.y, origin_pose->pose.orientation.z, origin_pose->pose.orientation.w);
  tf2::Quaternion q2(target_pose->pose.orientation.x, target_pose->pose.orientation.y, target_pose->pose.orientation.z, target_pose->pose.orientation.w);

  tf2::Matrix3x3 m1(q1);
  tf2::Matrix3x3 m2(q2);
  double roll_1, pitch_1, yaw_1;
  double roll_2, pitch_2, yaw_2;
  m1.getRPY(roll_1, pitch_1, yaw_1);
  m2.getRPY(roll_2, pitch_2, yaw_2);

  error.angular_x = get_shortest_angle_difference(roll_2, roll_1);
  error.angular_y = get_shortest_angle_difference(pitch_2, pitch_1);
  error.angular_z = get_shortest_angle_difference(yaw_2, yaw_1);

  return true;
}

bool transform_pose(const geometry_msgs::msg::PoseStamped::SharedPtr reference_pose,
                    const geometry_msgs::msg::PoseStamped::SharedPtr target_pose,
                    std::unique_ptr<tf2_ros::Buffer> &tf_buffer_,
                    geometry_msgs::msg::PoseStamped &pose_out_)
{
  try
  {

    tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(*target_pose, pose_out_, reference_pose->header.frame_id, tf2::Duration(std::chrono::seconds(1)));
    return true;
  }
  catch (const tf2::TransformException &ex)
  {
    // Log the Error.
    // RCLCPP_INFO(
    //     this->get_logger(), "Could not transform %s to %s: %s",
    //     toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return false;
  }
  return false;
}

// offset not added

float get_shortest_angle_difference(float angle_1, float angle_2)
{
  return std::atan2(std::sin(angle_1 - angle_2), std::cos(angle_1 - angle_2));
}
