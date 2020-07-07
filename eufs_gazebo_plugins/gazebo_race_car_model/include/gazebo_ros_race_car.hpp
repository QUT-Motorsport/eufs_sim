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
#include <ros/ros.h>

// Gazebo Includes
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

// ROS RACE CAR PLUGIN
#include "vehicle_model.hpp"
#include "../src/models/dynamic_bicycle.cpp"
#include "../src/models/point_mass.cpp"

namespace gazebo {

class RaceCarModelPlugin : public ModelPlugin {
public:
  RaceCarModelPlugin();

  ~RaceCarModelPlugin() override;

  void Reset() override;

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

  boost::shared_ptr<ros::NodeHandle> rosnode;

  physics::WorldPtr world;

  physics::ModelPtr model;

  transport::NodePtr gznode;

private:
  void update();

  double update_rate_;

  event::ConnectionPtr updateConnection;

  eufs::VehicleModelPtr vehicle;

  common::Time lastSimTime;

  transport::PublisherPtr worldControlPub;
};

} // namespace gazebo
#endif // GAZEBO_ROS_RACE_CAR_HPP
