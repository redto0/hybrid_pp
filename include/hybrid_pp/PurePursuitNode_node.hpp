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
    
    /// raduis for path point intersection
    float look_ahead_distance;

    /// target x and y for goal pose
    float target_point_x_, target_point_y;

    /// pure persuit alpha (look it up, kinda weird)
    float alpha;

    /// current speed from odom
    float current_speed;

    /// the speed to be set based off the eq v = sqrt(static_friction * gravity * turn_radius)
    float set_speed;

    // Parameters
    tf2::Duration transform_tolerance;
    float min_look_ahead_distance;
    float max_look_ahead_distance;

    /// constant to multiply by speed for look ahead distance calculation
    float k_dd;

    std::string rear_axle_frame;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Pub Sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr nav_ack_vel_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_marker_pub;

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    // subscriber callback
    /// callback for sending ackerman data after calculating ackerman angle
    void ackerman_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    /// callback for getting current speed from odom
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
};
