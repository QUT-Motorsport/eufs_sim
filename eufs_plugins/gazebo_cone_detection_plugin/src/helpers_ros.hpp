#include "rclcpp/rclcpp.hpp"

bool is_initalised(rclcpp::PublisherBase::SharedPtr publisher) {
    return (bool)publisher;
}

bool has_subscribers(rclcpp::PublisherBase::SharedPtr publisher) {
    return is_initalised(publisher) && publisher->get_subscription_count() > 0;
}
