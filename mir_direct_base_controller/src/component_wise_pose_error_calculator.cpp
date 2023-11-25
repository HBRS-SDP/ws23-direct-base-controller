#include "component_wise_pose_error_calculator.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
bool get_component_wise_pose_error(const geometry_msgs::msg::PoseStamped::SharedPtr pose_1, 
                                   const geometry_msgs::msg::PoseStamped::SharedPtr pose_2,
                                   ComponentWiseCartesianDifference &error)
{
    if (!pose_1 || !pose_2)
    {
        return false;   /* code */
    }

    error.linear_x = pose_2->pose.position.x - pose_1->pose.position.x;
    error.linear_y = pose_2->pose.position.y - pose_1->pose.position.y;
    error.linear_z = pose_2->pose.position.z - pose_1->pose.position.z;

    tf2::Quaternion q1(pose_1->pose.orientation.x, pose_1->pose.orientation.y, pose_1->pose.orientation.z, pose_1->pose.orientation.w);
    tf2::Quaternion q2(pose_2->pose.orientation.x, pose_2->pose.orientation.y, pose_2->pose.orientation.z, pose_2->pose.orientation.w);
    
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
                    geometry_msgs::msg::PoseStamped::SharedPtr transform_pose)
{
    try
        {
            rclcpp::Time common_time = tf_buffer_->getLatestCommonTime(
                target_pose.header.frame_id, reference_pose.header.frame_id);

            tf_buffer_->waitForTransform(
                target_pose.header.frame_id, reference_pose.header.frame_id,
                common_time, rclcpp::Duration::from_seconds(wait_for_transform_));

            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(target_pose, transformed_pose,
                             tf_buffer_->lookupTransform(
                                 reference_pose.header.frame_id, target_pose.header.frame_id, common_time));

            return transformed_pose;
        }
    catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(rclcpp::get_logger("YourClass"), "Transform failed: %s", ex.what());
            return geometry_msgs::msg::PoseStamped();  // or you can return a special value to indicate failure
        }
}

//offset not added

float get_shortest_angle_difference(float angle_1, float angle_2)
{
    return  std::atan2(std::sin(angle_1 - angle_2), std::cos(angle_1 - angle_2));
}










