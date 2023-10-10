#include "hybrid_pp/PurePursuitNode_node.hpp"

// For _1
using namespace std::placeholders;

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options) : Node("PurePursuitNode", options) {
    // Parameters
    float x = this->declare_parameter<float>("foo", -10.0);

    // Pub Sub
    sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&PurePursuitNode::ackerman_cb, this, _1));
    pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/nav_ack_vel", 1);

    // Log a sample log
    RCLCPP_INFO(this->get_logger(), "You passed %f", x);

}

void PurePursuitNode::ackerman_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Log a sample log
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->pose.position.x);

    // Create a message to publish
    auto ack_msg = ackermann_msgs::msg::AckermannDrive();
    ack_msg.steering_angle = 0.0;
    ack_msg.speed = 0.0;

    // Publish the message
    pub->publish(ack_msg);
    
}
