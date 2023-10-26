#pragma once

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PurePursuitNode : public rclcpp::Node {
private:
    // Var
    float look_ahead_distance_;
    float target_point_x_, target_point_y_;
    float alpha_;
    float speed_;

    // Parameters
    tf2::Duration transform_tolerance_;
    float min_look_ahead_distance_;
    float max_look_ahead_distance_;
    float k_dd_;
    std::string rear_axle_frame_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Pub Sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr nav_ack_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_marker_pub_;

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void ackerman_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
};
