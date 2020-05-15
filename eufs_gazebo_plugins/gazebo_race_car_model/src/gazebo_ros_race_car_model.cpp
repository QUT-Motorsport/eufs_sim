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
#include "gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

namespace gazebo {

RaceCarModelPlugin::RaceCarModelPlugin() {
  int  argc  = 0;
  char *argv = nullptr;
  ros::init(argc, &argv, "RaceCarModelPlugin");
  this->rosnode = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
}

RaceCarModelPlugin::~RaceCarModelPlugin() {
  this->updateConnection.reset();
}

void RaceCarModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  ROS_INFO("Loading RaceCarModelPlugin");
  ROS_INFO("RaceCarModelPlugin loading params");

  this->model = _model;

  this->world = this->model->GetWorld();

  this->gznode = transport::NodePtr(new transport::Node());

  this->gznode->Init();

  std::string vehicle_model_ = "";
  if (!_sdf->HasElement("vehicle_model")) {
    vehicle_model_ = "KinematicBicycle";
  } else {
    vehicle_model_ = _sdf->GetElement("vehicle_model")->Get<std::string>();
  }

  if (vehicle_model_ == "PointMass") {
    this->vehicle = std::unique_ptr<fssim::VehicleModel>(
      new fssim::PointMass(_model, _sdf, this->rosnode, this->gznode));
  } else if (vehicle_model_ == "KinematicBicycle") {
    this->vehicle = std::unique_ptr<fssim::VehicleModel>(
      new fssim::KinematicBicycle(_model, _sdf, this->rosnode, this->gznode));
  } else if (vehicle_model_ == "DynamicBicycle") {
    this->vehicle = std::unique_ptr<fssim::VehicleModel>(
      new fssim::VehicleModel(_model, _sdf, this->rosnode, this->gznode));
  } else {
    this->vehicle = std::unique_ptr<fssim::VehicleModel>(
      new fssim::VehicleModel(_model, _sdf, this->rosnode, this->gznode));
  }
  this->vehicle->printInfo();

  this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));

  this->worldControlPub = this->gznode->Advertise<msgs::WorldControl>("~/world_control");

  this->lastSimTime = this->world->SimTime();
}

void RaceCarModelPlugin::Reset() {
  this->lastSimTime = 0;
}

void RaceCarModelPlugin::update() {
  common::Time curTime = this->world->SimTime();

  double dt = 0.0;
  if (!isLoopTime(curTime, dt)) {
    return;
  }

  this->lastSimTime = curTime;

  this->vehicle->update(dt);
}

bool RaceCarModelPlugin::isLoopTime(const common::Time &time, double &dt) {
  dt = (time - this->lastSimTime).Double();
  
  if (dt < 0.0) {
    this->Reset();
    return false;
  } else if (ignition::math::equal(dt, 0.0)) {
    return false;
  }
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)
} // namespace gazebo
