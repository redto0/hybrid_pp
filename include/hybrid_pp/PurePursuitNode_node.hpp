#pragma once

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "laser_geometry/laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

/// Pure pursuit command result, with components
struct CommandCalcResult {
    ackermann_msgs::msg::AckermannDrive command;
    /// Distance to the point on the path used
    float look_ahead_distance{};
    /// Radius of the arc being followed in the command
    double turning_radius{};
};

/// Pure pursuit command result, with components
struct PathCalcResult {
    /// Point on the path we are moving to
    geometry_msgs::msg::PoseStamped intersection_point{};
    /// Distance to the point on the path used
    float look_ahead_distance{};
};

class PurePursuitNode : public rclcpp::Node {
private:
    /// Standard point (0,0), this acts as the center of the rear axle
    const geometry_msgs::msg::Point zero{};

    /// Current speed from odom.
    float current_speed;

    // Parameters
    float min_look_ahead_distance;
    float max_look_ahead_distance;
    float max_speed;
    float wheel_base;
    float gravity_constant;
    std::string rear_axle_frame;
    /// Constant to multiply by speed for look ahead distance calculation.
    float k_dd;
    /// Publishes visualisations if true
    bool debug;

    // Multithreading
    /// Set to true to kill thread
    std::atomic_bool stop_token{false};
    /// Thread that performs the main loop
    std::thread work_thread;
    /// Node frequency
    rclcpp::WallRate rate;
    /// Path lock
    std::mutex spline_mtx;
    /// Odom lock
    std::mutex odom_mtx;

    sensor_msgs::msg::LaserScan::SharedPtr laser_scan;
    std::vector<geometry_msgs::msg::PoseStamped> objects;

    std::mutex path_mutex;
    std::mutex scan_mutex;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> path_spline;

    // Pub/Sub
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr nav_ack_vel_pub;

    // Visualization publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr travel_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_vis_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intersection_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planner_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_vis_pub;

    static float distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::hypot((float)p1.x - (float)p2.x, (float)p1.y - (float)p2.y);
    }

    /// Publishes a zero move command
    void publish_stop_command() {
        ackermann_msgs::msg::AckermannDrive stop{};
        this->nav_ack_vel_pub->publish(stop);
    }

    void avoid_box();

    std::optional<PathCalcResult> get_path_point(std::vector<geometry_msgs::msg::PoseStamped> path_spline);
    std::vector<geometry_msgs::msg::PoseStamped> get_path_spline(const nav_msgs::msg::Path& path);

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// Callback for receiving paths
    void path_cb(nav_msgs::msg::Path::SharedPtr msg);
    /// Callback for getting current speed from odom.
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
    /// Callback for getting getting and pre-prossesing LIDAR scans
    void lidar_scan_cb(sensor_msgs::msg::LaserScan::SharedPtr msg);
    /// Publishes markers visualising the pure pursuit geometry.
    void publish_visualisation(CommandCalcResult, geometry_msgs::msg::PoseStamped intersection_point,
                               std::vector<geometry_msgs::msg::PoseStamped> spline);
    /// Calculates the command to reach the given point.
    CommandCalcResult calculate_command_to_point(const geometry_msgs::msg::PoseStamped& target_point,
                                                 float look_ahead_distance) const;

    ~PurePursuitNode() override { this->stop_token.store(true); }
};
