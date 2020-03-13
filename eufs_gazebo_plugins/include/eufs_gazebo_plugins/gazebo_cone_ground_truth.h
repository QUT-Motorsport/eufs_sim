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
 * @file gazebo_cone_ground_truth.h
 * @author Niklas Burggraaff <s1902977@ed.ac.uk>
 * @date Mar 10, 2020
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief ground truth cone Gazebo plugin
 *
 * @details TODO:
 * Provides ground truth state in simulation in the form of nav_msgs/Odometry and
 * eufs_msgs/CarState. Additionally can publish transform.P
 **/


#ifndef EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H
#define EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H

#include <math.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>

#include <eufs_msgs/ConeArray.h>
#include <eufs_msgs/CarState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace gazebo {

  class GazeboConeGroundTruth : public ModelPlugin {

  public:

    enum ConeType {
      blue, yellow, orange, big_orange
    };

    GazeboConeGroundTruth();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void UpdateChild();

    void addConeToConeArray(eufs_msgs::ConeArray &cone_array, physics::LinkPtr link);

    std::vector <geometry_msgs::Point> processCones(std::vector <geometry_msgs::Point> points);

    GazeboConeGroundTruth::ConeType getConeType(physics::LinkPtr link);


    physics::ModelPtr track_model;
    eufs_msgs::ConeArray cone_array_message;

    physics::LinkPtr car_link;
    ignition::math::Pose3d car_pos;

    float view_distance;
    float fov;

    event::ConnectionPtr update_connection_;

    ros::NodeHandle *rosnode_;

    ros::Publisher cone_pub_;
    ros::Publisher cone_marker_pub_;

    int addConeMarkers(std::vector <visualization_msgs::Marker> &marker_array, int marker_id,
                       std::vector <geometry_msgs::Point> cones, float red, float green, float blue, bool big);

  };
}

#endif //EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H
