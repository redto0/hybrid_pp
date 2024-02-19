#include "hybrid_pp/filters.hpp"

ackermann_msgs::msg::AckermannDrive MovingAverageFilter::update(const ackermann_msgs::msg::AckermannDrive& command) {
    this->buffer.push_back(command);

    if (this->window < this->buffer.size()) {
        this->buffer.pop_front();
    }

    float acc_speed = 0;
    float acc_steer = 0;

    for (const auto& item : this->buffer) {
        acc_speed += item.speed;
        acc_steer += item.steering_angle;
    }

    acc_steer /= this->buffer.size();
    acc_speed /= this->buffer.size();

    ackermann_msgs::msg::AckermannDrive ret{};
    ret.speed = acc_speed;
    ret.steering_angle = acc_steer;
    return ret;
}
