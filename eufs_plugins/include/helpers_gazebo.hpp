#pragma once

#include <string>
#include <optional>
#include <stdexcept>

#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_plugins {
namespace eufs_plugins {

inline double calc_dt(const std::chrono::steady_clock::time_point &start,
                      const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double>(end - start).count();
}

inline gz::sim::Model getModel(gz::sim::World &world, 
                        gz::sim::EntityComponentManager &ecm,
                        std::string name,
                        std::optional<const rclcpp::Logger> logger = std::nullopt) 
    {
        auto modelEntity = world.ModelByName(ecm, name);
        gz::sim::Model model(modelEntity);
        // this needs to be updated.
        // if (model == nullptr) 
        // {
        //     if (logger) 
        //     {
        //         RCLCPP_FATAL(*logger, "Could not find required model <%s>. Exiting.", name.c_str());
        //     }
        //     throw std::runtime_error("Model not found: " + name);
        // }
        return model;
    }

inline gz::sim::Link get_link(gz::sim::Model &model, 
                       gz::sim::EntityComponentManager &ecm,
                       std::string &name,
                       std::optional<const rclcpp::Logger> logger = std::nullopt) {
    auto linkEntity = model.LinkByName(ecm, name);
    gz::sim::Link link(linkEntity);
    
    if (!link.Valid(ecm)) 
    {
        if (logger) 
        {
            RCLCPP_FATAL(*logger, "Could not find required link <%s> on model <%s>. Exiting.", 
            name.c_str(), model.Name(ecm).c_str());
        }
        throw std::runtime_error("Model not found: " + name);
    }
    return link;
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins