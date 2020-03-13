/*MIT License
*
* Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
*         of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
*         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*         copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
*         copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.*/

/**
 * @file gazebo_cone_ground_truth.cpp
 * @author Niklas Burggraaff <s1902977@ed.ac.uk>
 * @date Mar 10, 2020
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief ground truth cone Gazebo plugin
 *
 * @details TODO:
 * Provides ground truth state in simulation in the form of nav_msgs/Odometry and
 * eufs_msgs/CarState. Additionally can publish transform.P
 **/

#include "../include/eufs_gazebo_plugins/gazebo_cone_ground_truth.h"

namespace gazebo {

  GZ_REGISTER_MODEL_PLUGIN(GazeboConeGroundTruth)

  GazeboConeGroundTruth::GazeboConeGroundTruth() {
  }

  void GazeboConeGroundTruth::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // TODO(Niklas): This needs to be different for different gazebo versions
    this->track_model = _parent->GetWorld()->ModelByName("track");
    this->car_link = _parent->GetLink("base_footprint");

    this->view_distance = 15;
    this->fov = 1.91986;

    this->rosnode_ = new ros::NodeHandle("~");

    this->cone_pub_ = this->rosnode_->advertise<eufs_msgs::ConeArray>("/ground_truth/cones_test", 1);
    this->cone_marker_pub_ = this->rosnode_->advertise<visualization_msgs::MarkerArray>("/ground_truth/cones_test/viz", 1);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboConeGroundTruth::UpdateChild, this));
  }

  void GazeboConeGroundTruth::UpdateChild() {
    if (ros::Time::now().toSec() - cone_array_message.header.stamp.toSec() < (1.0 / 25.0)) {
      return;
    }

    eufs_msgs::ConeArray new_cone_array_message;

    this->car_pos = this->car_link->WorldPose();

    physics::Link_V links = this->track_model->GetLinks();
    for (unsigned int i = 0; i < links.size(); i++) {
      addConeToConeArray(new_cone_array_message, links[i]);
    }

    this->cone_array_message.blue_cones = processCones(new_cone_array_message.blue_cones);
    this->cone_array_message.yellow_cones = processCones(new_cone_array_message.yellow_cones);
    this->cone_array_message.orange_cones = processCones(new_cone_array_message.orange_cones);
    this->cone_array_message.big_orange_cones = processCones(new_cone_array_message.big_orange_cones);

    cone_array_message.header.frame_id = "/base_footprint";
    cone_array_message.header.stamp = ros::Time::now();

    this->cone_pub_.publish(this->cone_array_message);

    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    marker_id = addConeMarkers(marker_array.markers, marker_id, cone_array_message.blue_cones, 0.2, 0.2, 1, false);
    marker_id = addConeMarkers(marker_array.markers, marker_id, cone_array_message.yellow_cones, 1, 1, 0, false);
    marker_id = addConeMarkers(marker_array.markers, marker_id, cone_array_message.orange_cones, 1, 0.549, 0, false);
    marker_id = addConeMarkers(marker_array.markers, marker_id, cone_array_message.big_orange_cones, 1, 0.271, 0, true);

    this->cone_marker_pub_.publish(marker_array);
  }

  int GazeboConeGroundTruth::addConeMarkers(std::vector<visualization_msgs::Marker> &marker_array, int marker_id, std::vector<geometry_msgs::Point> cones, float red, float green, float blue, bool big) {
    int id = marker_id;
    for (int i = 0; i < cones.size(); i++) {
      visualization_msgs::Marker marker;

      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "/base_footprint";

      marker.id = id;

      marker.type = marker.MESH_RESOURCE;
      marker.action = marker.ADD;

      marker.pose.position.x = cones[i].x;
      marker.pose.position.y = cones[i].y;

      marker.scale.x = 1.5;
      marker.scale.y = 1.5;
      marker.scale.z = 1.5;

      if (big) {
        marker.mesh_resource = "package://eufs_description/meshes/cone_big.dae";
      } else {
        marker.mesh_resource = "package://eufs_description/meshes/cone.dae";
      }

      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration(0.2);

      marker_array.push_back(marker);

      id++;
    }
    return id;
  }

  std::vector<geometry_msgs::Point> GazeboConeGroundTruth::processCones(std::vector<geometry_msgs::Point> points) {
    std::vector<geometry_msgs::Point> points_in_view;

    for (unsigned int i = 0; i < points.size(); i++) {
      // Translate the position of the cone to be based on the car
      float x = points[i].x - this->car_pos.Pos().X();
      float y = points[i].y - this->car_pos.Pos().Y();

      if ((x * x) + (y * y) < (view_distance * view_distance)) {
        // Rotate the points using the yaw of the car
        float yaw = this->car_pos.Rot().Yaw() * (-1);

        points[i].x = (cos(yaw) * x) - (sin(yaw) * y);
        points[i].y = (sin(yaw) * x) + (cos(yaw) * y);

        float angle = atan2(points[i].y, points[i].x);

        if (abs(angle) < (this->fov / 2)) {
          points_in_view.push_back(points[i]);
        }
      }
    }

    return points_in_view;
  }

  void GazeboConeGroundTruth::addConeToConeArray(eufs_msgs::ConeArray &cone_array, physics::LinkPtr link) {
    geometry_msgs::Point point;
    // TODO(Niklas): This needs to be different for different gazebo versions
    point.x = link->WorldPose().Pos().X();
    point.y = link->WorldPose().Pos().Y();
    point.z = 0;

    ConeType cone_type = this->getConeType(link);

    switch (cone_type) {
      case ConeType::blue:
        cone_array.blue_cones.push_back(point);
        break;
      case ConeType::yellow:
        cone_array.yellow_cones.push_back(point);
        break;
      case ConeType::orange:
        cone_array.orange_cones.push_back(point);
        break;
      case ConeType::big_orange:
        cone_array.big_orange_cones.push_back(point);
        break;
    }
  }

  GazeboConeGroundTruth::ConeType GazeboConeGroundTruth::getConeType(physics::LinkPtr link) {
    std::string link_name = link->GetName();

    if (link_name.substr(0, 9) == "blue_cone") {
      return ConeType::blue;
    }
    if (link_name.substr(0, 11) == "yellow_cone") {
      return ConeType::yellow;
    }
    if (link_name.substr(0, 11) == "orange_cone") {
      return ConeType::orange;
    }
    if (link_name.substr(0, 8) == "big_cone") {
      return ConeType::big_orange;
    }

    ROS_ERROR("Unknown link in model: %s", link_name.c_str());
  }

}