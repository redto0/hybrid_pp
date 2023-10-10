#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PurePursuitNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub;

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void ackerman_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg);
};
