// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_LIDAR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_LIDAR_HPP_

// Gazebo Includes
#include <gz/sensors/Lidar.hh>
#include <sdf/Sensor.hh>

// C++ includes
#include <memory>
#include <string>

// RCLCPP
#include <rclcpp/rclcpp.hpp>

namespace gazebo_plugins {

class GazeboRosLidarPrivate;

class GazeboRosLidar : public gz::sensors::Lidar {
   public:
    /// \brief Constructor
    GazeboRosLidar();

    /// \brief Destructor
    virtual ~GazeboRosLidar();

    /// \brief Runs on sensor load
    bool Load(const sdf::Sensor &sdfSensor) override;


   private:
    std::unique_ptr<GazeboRosLidarPrivate> impl_; 
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_LIDAR_HPP_
