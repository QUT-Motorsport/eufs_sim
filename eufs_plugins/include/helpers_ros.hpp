#include "rclcpp/rclcpp.hpp"

bool is_initalised(rclcpp::PublisherBase::SharedPtr publisher) { return (bool)publisher; }

bool has_subscribers(rclcpp::PublisherBase::SharedPtr publisher) {
    return is_initalised(publisher) && publisher->get_subscription_count() > 0;
}

std::vector<double> to_quaternion(std::vector<double> &euler) {
    // Abbreviations for the various angular functions
    double cy = cos(euler[0] * 0.5);
    double sy = sin(euler[0] * 0.5);
    double cp = cos(euler[1] * 0.5);
    double sp = sin(euler[1] * 0.5);
    double cr = cos(euler[2] * 0.5);
    double sr = sin(euler[2] * 0.5);

    std::vector<double> q;
    q.push_back(cy * cp * sr - sy * sp * cr);  // x
    q.push_back(sy * cp * sr + cy * sp * cr);  // y
    q.push_back(sy * cp * cr - cy * sp * sr);  // z
    q.push_back(cy * cp * cr + sy * sp * sr);  // w

    return q;
}
