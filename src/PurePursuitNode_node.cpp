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
    current_speed = 1;
    set_speed = 0;

    // Pub Sub
    goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1, std::bind(&PurePursuitNode::ackerman_cb, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_can", 1,
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

    // Calculate the angle between the robot and the goal.
    alpha = std::atan2(transformed_goal_pose.pose.pose.position.y, transformed_goal_pose.pose.pose.position.x);

    // Set the set the desired steering angle and set it to the ackerman message
    steering_angle = std::atan(2.0 * wheel_base * std::sin(alpha) / look_ahead_distance);
    ack_msg.steering_angle = steering_angle;

    // Gets the distance to the instantanious center of rotation from the center of the rear axle
    double distance_to_icr = wheel_base / std::tan(steering_angle);

    // Set the speed based off the eq v = sqrt(static_friction * gravity * turn_radius) with static friction being 1.
    set_speed = std::sqrt(gravity_constant * std::abs(distance_to_icr));
    ack_msg.speed = set_speed;

    // Publish the ackerman message
    nav_ack_vel_pub->publish(ack_msg);

    // Declares the path prediciton marker in the rear axle frame
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

    // Declares the look ahead marker in the rear axle frame
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

    // Standard point (0,0), this acts as the center of the rear axle
    geometry_msgs::msg::Point zero;

    // If our steering angle is negative, graphing math is a little different
    if (steering_angle < 0) {
        // Loop runs until the point generated hits the look ahead distance raduis
        for (double index = 0; distance(graph_point, zero) - look_ahead_distance <= 0; index += 0.01) {
            // Sets the x and y coords using ploar coodiants the the ICR as the origin
            graph_point.x = distance_to_icr * -std::sin(index);
            graph_point.y = distance_to_icr * -std::cos(index) + distance_to_icr;

            // Appeneds the generated point to the markers list of points
            path_prediction_marker.points.push_back(graph_point);
        }
    } else {
        // Loop runs until the point generated hits the look ahead distance raduis
        for (double index = -3.14 / 2; distance(graph_point, zero) - look_ahead_distance <= 0; index += 0.01) {
            // Sets the x and y coords using ploar coodiants the the ICR as the origin
            graph_point.x = distance_to_icr * std::cos(index);
            graph_point.y = distance_to_icr * std::sin(index) + distance_to_icr;

            // Appeneds the generated point to the markers list of points
            path_prediction_marker.points.push_back(graph_point);
        }
    }

    // Appeneds a list of points in a circle to the look ahead distance marker using look ahead distance as raduis
    for (double index = 0; index <= 2 * 3.14; index += 0.01) {
        graph_point.x = look_ahead_distance * std::cos(index);
        graph_point.y = look_ahead_distance * std::sin(index);

        look_ahead_distance_marker.points.push_back(graph_point);
    }

    // Publish the markers to the visualization_marker topic
    path_vis_marker_pub->publish(path_prediction_marker);
    path_vis_marker_pub->publish(look_ahead_distance_marker);

    // Clear the points that were published so new points can be displayed and we dont get messy overlap
    path_prediction_marker.points.clear();
    look_ahead_distance_marker.points.clear();
}

void PurePursuitNode::odom_speed_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Gets the current speed of the cart
    current_speed = 1;// msg->twist.twist.linear.x;
}