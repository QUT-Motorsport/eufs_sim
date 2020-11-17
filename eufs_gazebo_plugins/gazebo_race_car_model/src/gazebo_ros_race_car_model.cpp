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

// Main Include
#include "gazebo_race_car_model/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

namespace gazebo_plugins {
namespace eufs {

RaceCarModelPlugin::RaceCarModelPlugin() {
}

RaceCarModelPlugin::~RaceCarModelPlugin() {
  this->updateConnection.reset();
}

void RaceCarModelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->rosnode = gazebo_ros::Node::Get(_sdf);

  RCLCPP_DEBUG(this->rosnode->get_logger(), "Loading RaceCarModelPlugin");

  this->model = _model;

  this->world = this->model->GetWorld();

  this->gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());

  this->gznode->Init();

  // Get the vehicle model from the sdf
  std::string vehicle_model_ = "";
  if (!_sdf->HasElement("vehicle_model")) {
    vehicle_model_ = "DynamicBicycle";
  } else {
    vehicle_model_ = _sdf->GetElement("vehicle_model")->Get<std::string>();
  }

  if (vehicle_model_ == "PointMass") {
    this->vehicle = std::unique_ptr<eufs::VehicleModel>(
      new eufs::PointMass(_model, _sdf, this->rosnode, this->gznode));
  } else if (vehicle_model_ == "DynamicBicycle") {
    this->vehicle = std::unique_ptr<eufs::VehicleModel>(
      new eufs::DynamicBicycle(_model, _sdf, this->rosnode, this->gznode));
  } else {
    this->vehicle = std::unique_ptr<eufs::VehicleModel>(
      new eufs::VehicleModel(_model, _sdf, this->rosnode, this->gznode));
  }
  this->vehicle->printInfo();

  // Get the update rate from the sdf
  if (!_sdf->HasElement("update_rate")) {
    this->update_rate_ = 1000.0;
  } else {
    this->update_rate_ = _sdf->GetElement("update_rate")->Get<double>();
  }

  this->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));

  this->worldControlPub = this->gznode->Advertise<gazebo::msgs::WorldControl>("~/world_control");

#if GAZEBO_MAJOR_VERSION >= 8
  this->lastSimTime = this->world->SimTime();
#else
  this->lastSimTime = this->world->GetSimTime();
#endif

  RCLCPP_INFO(this->rosnode->get_logger(), "RaceCarModelPlugin Loaded");
}

void RaceCarModelPlugin::Reset() {
  this->lastSimTime = 0;
}

void RaceCarModelPlugin::update() {
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time curTime = this->world->SimTime();
#else
  gazebo::common::Time curTime = this->world->GetSimTime();
#endif

  double dt = (curTime - this->lastSimTime).Double();
  if (dt < (1 / this->update_rate_)) {
    return;
  }

  this->lastSimTime = curTime;

  this->vehicle->update(dt);
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)

} // namespace eufs
} // namespace gazebo_plugins