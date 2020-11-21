/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef GAZEBO_ROS_RACE_CAR_HPP
#define GAZEBO_ROS_RACE_CAR_HPP

// ROS Includes
#include "rclcpp/rclcpp.hpp"

// Gazebo Includes
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

// ROS RACE CAR PLUGIN
#include "vehicle_model.hpp"
#include "../../src/models/dynamic_bicycle.cpp"
#include "../../src/models/point_mass.cpp"

namespace gazebo_plugins {
namespace eufs {

class RaceCarModelPlugin : public gazebo::ModelPlugin {
 public:
  RaceCarModelPlugin();

  ~RaceCarModelPlugin() override;

  void Reset() override;

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

  std::shared_ptr<rclcpp::Node> rosnode;

  gazebo::physics::WorldPtr world;

  gazebo::physics::ModelPtr model;

  gazebo::transport::NodePtr gznode;

 private:
  void update();

  double update_rate_;

  gazebo::event::ConnectionPtr updateConnection;

  eufs::VehicleModelPtr vehicle;

  gazebo::common::Time lastSimTime;

  gazebo::transport::PublisherPtr worldControlPub;
};

} // namespace eufs
} // namespace gazebo_plugins
#endif // GAZEBO_ROS_RACE_CAR_HPP
