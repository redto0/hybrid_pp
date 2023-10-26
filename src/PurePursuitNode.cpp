#include "hybrid_pp/PurePursuitNode_node.hpp"

int main(int argc, char** argv) {
    // Setup runtime
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add nodes to executor
    auto node = std::make_shared<PurePursuitNode>(options);
    exec.add_node(node);

    // Run
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
