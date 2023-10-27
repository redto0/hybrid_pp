#include "hybrid_pp/PurePursuitNode_node.hpp"

// For _1
using namespace std::placeholders;

// For 1000ms
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options) : Node("PurePursuitNode", options) {
    // Params
    min_look_ahead_distance = this->declare_parameter<float>("min_look_ahead_distace", 3.0);
    max_look_ahead_distance = this->declare_parameter<float>("max_look_ahead_distace", 10.0);
    k_dd = this->declare_parameter<float>("k_dd", 0.4);
    rear_axle_frame = "rear_axle";  // this->declare_parameter<std::string>("rear_axle_frame", "rear_axle");
    wheel_base = this->declare_parameter<float>("wheel_base", 1.08);
    gravity_constant = this->declare_parameter<float>("gravity_constant", 9.81);

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

void PurePursuitNode::ackerman_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Declare the goal pose var with the incomming msg
    msg_to_goal_pose.header.frame_id = msg->header.frame_id;
    msg_to_goal_pose.header.stamp = this->get_clock()->now();
    msg_to_goal_pose.pose.pose.position.x = msg->pose.position.x;
    msg_to_goal_pose.pose.pose.position.y = msg->pose.position.y;
    msg_to_goal_pose.pose.pose.position.z = msg->pose.position.z;

    // Transform goal pose to rear_axle frame
    transformed_goal_pose = tf_buffer->transform(msg_to_goal_pose, rear_axle_frame, transform_tolerance);

    // Calc look ahead distance
    look_ahead_distance = std::clamp(k_dd * current_speed, min_look_ahead_distance, max_look_ahead_distance);

    // Calculate the angle between the robot and the goal
    alpha = std::atan2(transformed_goal_pose.pose.pose.position.y, transformed_goal_pose.pose.pose.position.x);

    // Calculate the steering angle (needs wheel base) TODO
    steering_angle = std::atan(2.0 * wheel_base * std::sin(alpha) / look_ahead_distance);

    // Create the ackermann message
    ackermann_msgs::msg::AckermannDrive ack_msg;
    ack_msg.steering_angle = steering_angle;

    auto theta = steering_angle;
    double radius = wheel_base / std::tan(theta);

    set_speed = std::sqrt(gravity_constant * std::abs(radius));
    ack_msg.speed = set_speed;

    // Publish the message
    nav_ack_vel_pub->publish(ack_msg);

    path_prediction_marker.header.frame_id = "/rear_axle";
    path_prediction_marker.header.stamp = this->get_clock()->now();
    path_prediction_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_prediction_marker.action = visualization_msgs::msg::Marker::ADD;
    path_prediction_marker.ns = "hybrid_pp_ns";
    path_prediction_marker.id = 0;
    path_prediction_marker.pose.orientation.w = 1.0;
    path_prediction_marker.scale.x = 0.1;
    path_prediction_marker.color.a = 1.0;
    path_prediction_marker.color.g = 1.0;

    look_ahead_distance_marker.header.frame_id = "/rear_axle";
    look_ahead_distance_marker.header.stamp = this->get_clock()->now();
    look_ahead_distance_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    look_ahead_distance_marker.action = visualization_msgs::msg::Marker::ADD;
    look_ahead_distance_marker.ns = "hybrid_pp_ns";
    look_ahead_distance_marker.id = 1;
    look_ahead_distance_marker.pose.orientation.w = 1.0;
    look_ahead_distance_marker.scale.x = 0.1;
    look_ahead_distance_marker.color.a = 1.0;
    look_ahead_distance_marker.color.r = 1.0;

    if (theta < 0) {
        for (double index = 0; distance_from_rear_axle(graph_point) - look_ahead_distance <= 0; index += 0.01) {
            graph_point.pose.pose.position.x = radius * -std::sin(index);
            graph_point.pose.pose.position.y = radius * -std::cos(index) + radius;

            path_prediction_marker.points.push_back(graph_point.pose.pose.position);
        }
    } else {
        for (double index = -3.14 / 2; distance_from_rear_axle(graph_point) - look_ahead_distance <= 0; index += 0.01) {
            graph_point.pose.pose.position.x = radius * std::cos(index);
            graph_point.pose.pose.position.y = radius * std::sin(index) + radius;

            path_prediction_marker.points.push_back(graph_point.pose.pose.position);
        }
    }

    for (double index = 0; index <= 2 * 3.14; index += 0.1) {
        graph_point.pose.pose.position.x = look_ahead_distance * std::cos(index);
        graph_point.pose.pose.position.y = look_ahead_distance * std::sin(index);

        look_ahead_distance_marker.points.push_back(graph_point.pose.pose.position);
    }

    path_vis_marker_pub->publish(path_prediction_marker);
    path_vis_marker_pub->publish(look_ahead_distance_marker);

    path_prediction_marker.points.clear();
    look_ahead_distance_marker.points.clear();
}

void PurePursuitNode::odom_speed_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_speed = msg->twist.twist.linear.x;
}

// distance function that works to geometry_msgs
auto distance = [](const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
};

float PurePursuitNode::distance_from_rear_axle(const geometry_msgs::msg::PoseWithCovarianceStamped& p1) {
    auto transformed_pose = tf_buffer->transform(p1, rear_axle_frame, transform_tolerance).pose.pose.position;

    return std::sqrt(std::pow(transformed_pose.x - 0, 2) + std::pow(transformed_pose.y - 0, 2));
}