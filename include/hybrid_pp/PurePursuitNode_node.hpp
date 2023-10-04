#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PurePursuitNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);
};
