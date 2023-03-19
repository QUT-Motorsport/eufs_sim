#pragma once

#include <gazebo/gazebo.hh>
#include <string>

#include "rclcpp/rclcpp.hpp"

double calc_dt(gazebo::common::Time start, gazebo::common::Time end) { return (end - start).Double(); }

gazebo::physics::ModelPtr get_model(gazebo::physics::WorldPtr world, std::string name,
                                    std::optional<const rclcpp::Logger> logger = {}) {
    gazebo::physics::ModelPtr model = world->ModelByName(name);
    if (model == nullptr) {
        if (logger) {
            RCLCPP_FATAL(*logger, "Could not find required model <%s>. Exiting.", name.c_str());
        }
        exit(1);
    }
    return model;
}

gazebo::physics::LinkPtr get_link(gazebo::physics::ModelPtr model, std::string name,
                                  std::optional<const rclcpp::Logger> logger = {}) {
    gazebo::physics::LinkPtr link = model->GetLink(name);
    if (link == nullptr) {
        if (logger) {
            RCLCPP_FATAL(*logger, "Could not find required link <%s> on model <%s>. Exiting.", name.c_str(),
                         model->GetName().c_str());
        }
        exit(1);
    }
    return link;
}

double get_double_parameter(sdf::ElementPtr sdf, std::string element, double default_value,
                            std::string default_description, std::optional<const rclcpp::Logger> logger = {}) {
    if (!sdf->HasElement(element)) {
        if (logger) {
            RCLCPP_WARN(*logger, "Plugin missing parameter <%s>, defaults to %s.", element.c_str(),
                        default_description.c_str());
        }
        return default_value;
    } else {
        return sdf->GetElement(element)->Get<double>();
    }
}

bool get_bool_parameter(sdf::ElementPtr sdf, std::string element, bool default_value, std::string default_description,
                        std::optional<const rclcpp::Logger> logger = {}) {
    if (!sdf->HasElement(element)) {
        if (logger) {
            RCLCPP_WARN(*logger, "Plugin missing parameter <%s>, defaults to %s.", element.c_str(),
                        default_description.c_str());
        }
        return default_value;
    } else {
        return sdf->GetElement(element)->Get<bool>();
    }
}

std::string get_string_parameter(sdf::ElementPtr sdf, std::string element, std::string default_value,
                                 std::string default_description, std::optional<const rclcpp::Logger> logger = {}) {
    if (!sdf->HasElement(element)) {
        if (logger) {
            RCLCPP_WARN(*logger, "Plugin missing parameter <%s>, defaults to %s.", element.c_str(),
                        default_description.c_str());
        }
        return default_value;
    } else {
        return sdf->GetElement(element)->Get<std::string>();
    }
}
