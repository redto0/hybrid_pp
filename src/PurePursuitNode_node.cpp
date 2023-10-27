#include "hybrid_pp/PurePursuitNode_node.hpp"

#include <algorithm>
#include <cmath>

// For _1
using namespace std::placeholders;

// For 1000ms
using namespace std::chrono_literals;

visualization_msgs::msg::Marker marker;
visualization_msgs::msg::Marker ld;

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options) : Node("PurePursuitNode", options) {
    // Params (tune theses)
    min_look_ahead_distance = this->declare_parameter<float>("min_look_ahead_distace", 3.0);
    max_look_ahead_distance = this->declare_parameter<float>("max_look_ahead_distace", 10.0);
    k_dd = this->declare_parameter<float>("k_dd", 0.2);
    rear_axle_frame = this->declare_parameter<std::string>("rear_axle_frame", "rear_axle");

    // Var init
    current_speed = 0;
    set_speed = 0;

    // Pub Sub
    goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1, std::bind(&PurePursuitNode::ackerman_cb, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                  std::bind(&PurePursuitNode::odom_speed_cb, this, _1));
    nav_ack_vel_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/nav_ack_vel", 1);
    path_vis_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

    // TF
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initial var state
    transform_tolerance = tf2::durationFromSec(0.1);
}

// base_link to odom
void PurePursuitNode::ackerman_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Log a sample log
    RCLCPP_INFO(this->get_logger(), "I heard the message");

    geometry_msgs::msg::PoseWithCovarianceStamped msg_to_goal_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped transformed_robot_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped transformed_goal_pose;

    msg_to_goal_pose.header.frame_id = msg->header.frame_id;
    msg_to_goal_pose.header.stamp = this->get_clock()->now();
    msg_to_goal_pose.pose.pose.position.x = msg->pose.position.x;
    msg_to_goal_pose.pose.pose.position.y = msg->pose.position.y;
    msg_to_goal_pose.pose.pose.position.z = msg->pose.position.z;

    transformed_goal_pose = tf_buffer->transform(msg_to_goal_pose, rear_axle_frame, transform_tolerance);

    // Calc look ahead distance
    look_ahead_distance = std::clamp(k_dd * current_speed, min_look_ahead_distance, max_look_ahead_distance);

    RCLCPP_INFO(this->get_logger(), "Goal pose x: '%f'", transformed_goal_pose.pose.pose.position.x);
    RCLCPP_INFO(this->get_logger(), "Goal pose y: '%f'", transformed_goal_pose.pose.pose.position.y);

    // distance function that works to geometry_msgs
    auto distance = [](const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    };

    float steering_angle_;

    // Calculate the angle between the robot and the goal
    alpha = std::atan2(transformed_goal_pose.pose.pose.position.y, transformed_goal_pose.pose.pose.position.x);

    // Calculate the steering angle (needs wheel base) TODO
    steering_angle_ = std::atan(2.0 * 1.08 * std::sin(alpha) / look_ahead_distance);

    // Create the ackermann message
    ackermann_msgs::msg::AckermannDrive ack_msg;
    ack_msg.steering_angle = steering_angle_;

    auto theta = steering_angle_;
    double radius = 1.08 / std::tan(theta);

    set_speed = std::sqrt(9.81 * std::abs(radius));
    ack_msg.speed = set_speed;

    // Publish the message
    nav_ack_vel_pub->publish(ack_msg);

    auto y1 = 2 * alpha;

    geometry_msgs::msg::Point graphPoint;

    marker.header.frame_id = "/rear_axle";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.g = 1.0;

    ld.header.frame_id = "/rear_axle";
    ld.header.stamp = this->get_clock()->now();
    ld.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ld.action = visualization_msgs::msg::Marker::ADD;
    ld.ns = "my_namespace";
    ld.id = 1;
    ld.pose.orientation.w = 1.0;
    ld.scale.x = 0.1;
    ld.color.a = 1.0;  // Don't forget to set the alpha!
    ld.color.r = 1.0;

    RCLCPP_INFO(this->get_logger(), "radius: '%f'", radius);
    RCLCPP_INFO(this->get_logger(), "steering angle: '%f'", theta);

    if (theta < 0) {
        for (double index = 0;
             distance(graphPoint, transformed_robot_pose.pose.pose.position) - look_ahead_distance <= 0;
             index += 0.01) {
            graphPoint.x = radius * -std::sin(index);
            graphPoint.y = radius * -std::cos(index) + radius;
            // RCLCPP_INFO(this->get_logger(), "graph point x: '%f'", graphPoint.x);
            // RCLCPP_INFO(this->get_logger(), "graph point y: '%f'", graphPoint.y);

            marker.points.push_back(graphPoint);
        }
    } else {
        for (double index = -3.14 / 2;
             distance(graphPoint, transformed_robot_pose.pose.pose.position) - look_ahead_distance <= 0;
             index += 0.01) {
            graphPoint.x = radius * std::cos(index);
            graphPoint.y = radius * std::sin(index) + radius;
            // RCLCPP_INFO(this->get_logger(), "graph point x: '%f'", graphPoint.x);
            // RCLCPP_INFO(this->get_logger(), "graph point y: '%f'", graphPoint.y);

            marker.points.push_back(graphPoint);
        }
    }

    for (double index = 0; index <= 2 * 3.14; index += 0.1) {
        graphPoint.x = look_ahead_distance * std::cos(index);
        graphPoint.y = look_ahead_distance * std::sin(index);
        // RCLCPP_INFO(this->get_logger(), "graph point x: '%f'", graphPoint.x);
        // RCLCPP_INFO(this->get_logger(), "graph point y: '%f'", graphPoint.y);

        ld.points.push_back(graphPoint);
    }

    path_vis_marker_pub->publish(marker);
    path_vis_marker_pub->publish(ld);

    marker.points.clear();
    ld.points.clear();
}

void PurePursuitNode::odom_speed_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_speed = msg->twist.twist.linear.x;
}
