#pragma once

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
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

    /// The ackerman message to be published to the /nav_ack_vel topic with steering angle and speed
    ackermann_msgs::msg::AckermannDrive ack_msg;

    /// Varibale to convert target pose message to a PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped msg_to_goal_pose;

    /// Used for the transformation of the goal pose from /odom frame to /rear_axle frame
    geometry_msgs::msg::PoseWithCovarianceStamped transformed_goal_pose;

    /// Visualization marker for the predicted path arc
    visualization_msgs::msg::Marker path_prediction_marker;

    /// Visualization marker for the raduis of the lookahead distance
    visualization_msgs::msg::Marker look_ahead_distance_marker;

    /// The steering angle that will be output after pure pursuit claculations.
    float steering_angle;

    /// Raduis for path point intersection.
    float look_ahead_distance;

    /// Target x and y for goal pose.
    float target_point_x_, target_point_y;

    /// Pure pursuit alpha (look it up, kinda weird).
    float alpha;

    /// Current speed from odom.
    float current_speed;

    /// The speed to be set based off the eq v = sqrt(static_friction * gravity * turn_radius).
    float set_speed;

    // Parameters
    tf2::Duration transform_tolerance;
    float min_look_ahead_distance;
    float max_look_ahead_distance;
    float wheel_base;
    float gravity_constant;
    std::string rear_axle_frame;

    /// Constant to multiply by speed for look ahead distance calculation.
    float k_dd;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Pub/Sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr nav_ack_vel_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_marker_pub;

    float distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    };

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);
    // Subscriber callback

    /// Callback for sending ackerman data after calculating ackerman angle.
    void ackerman_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    /// Callback for getting current speed from odom.
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
};
