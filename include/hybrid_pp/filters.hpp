#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "deque"

/// Filters commands using a moving average
class MovingAverageFilter {
    std::deque<ackermann_msgs::msg::AckermannDrive> buffer;
    uint32_t window;

public:
    explicit MovingAverageFilter(uint32_t window) : window(window) {}

    /// Updates the moving average filter, and outputs a new filtered value
    ackermann_msgs::msg::AckermannDrive update(const ackermann_msgs::msg::AckermannDrive& command);
};