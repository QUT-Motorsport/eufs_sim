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
 * @details Provides ground truth cones in simulation in the form of `eufs_msgs/ConeArrayWithCovariance`.
 * Can also simulate the perception stack by publishing cones with noise.
 **/


#ifndef EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H
#define EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
//#include <gazebo/common/Events.hh>
//#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_array.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <eufs_msgs/msg/point_array.hpp>
#include <eufs_msgs/msg/car_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>

namespace gazebo_plugins {
namespace eufs {

  class GazeboConeGroundTruth : public gazebo::ModelPlugin {

  public:

    enum ConeType {
      blue, yellow, orange, big_orange, unknown
    };

    GazeboConeGroundTruth();

    // Gazebo plugin functions

    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void UpdateChild();

    // Getting the track
    eufs_msgs::msg::ConeArrayWithCovariance getTrackMessage();

    // Getting the cone arrays
    eufs_msgs::msg::ConeArrayWithCovariance getConeArrayMessage(eufs_msgs::msg::ConeArrayWithCovariance &track);

    void addConeToConeArray(eufs_msgs::msg::ConeArrayWithCovariance &ground_truth_cone_array, gazebo::physics::LinkPtr link);

    void processCones(eufs_msgs::msg::ConeArrayWithCovariance &cones);

    std::pair<std::vector<eufs_msgs::msg::ConeWithCovariance>, std::vector<eufs_msgs::msg::ConeWithCovariance>>
      fovCones(std::vector<eufs_msgs::msg::ConeWithCovariance> conesToCheck);

    GazeboConeGroundTruth::ConeType getConeType(gazebo::physics::LinkPtr link);

    // Getting the cone marker array
    visualization_msgs::msg::MarkerArray getConeMarkerArrayMessage(eufs_msgs::msg::ConeArrayWithCovariance &cone_array_message);

    int addConeMarkers(std::vector <visualization_msgs::msg::Marker> &marker_array, int marker_id,
                       std::vector <eufs_msgs::msg::ConeWithCovariance> cones, float red, float green, float blue, bool big);

    // Add noise to the cone arrays
    eufs_msgs::msg::ConeArrayWithCovariance getConeArrayMessageWithNoise(eufs_msgs::msg::ConeArrayWithCovariance &ground_truth_cone_array_message, ignition::math::Vector3d noise);

    void addNoiseToConeArray(std::vector<eufs_msgs::msg::ConeWithCovariance> &cone_array, ignition::math::Vector3d noise);

    double GaussianKernel(double mu, double sigma);

      // Helper function for parameters
    bool getBoolParameter(sdf::ElementPtr _sdf, const char* element, bool default_value, const char* default_description);
    double getDoubleParameter(sdf::ElementPtr _sdf, const char* element, double default_value, const char* default_description);
    std::string getStringParameter(sdf::ElementPtr _sdf, const char* element, std::string default_value, const char* default_description);
    ignition::math::Vector3d getVector3dParameter(sdf::ElementPtr _sdf, const char* element, ignition::math::Vector3d default_value, const char* default_description);

    // Strip away covariance
    eufs_msgs::msg::ConeArray stripCovariance(eufs_msgs::msg::ConeArrayWithCovariance msg);

    // Helper functions for determining whether a cone is in range
    bool inRangeOfCamera(eufs_msgs::msg::ConeWithCovariance cone);
    bool inFOVOfCamera(eufs_msgs::msg::ConeWithCovariance cone);
    bool inRangeOfLidar(eufs_msgs::msg::ConeWithCovariance cone);
    bool inFOVOfLidar(eufs_msgs::msg::ConeWithCovariance cone);

    // Publishers
    rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr ground_truth_cone_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ground_truth_cone_marker_pub_;

    rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr ground_truth_track_pub_;

    rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr perception_cone_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perception_cone_marker_pub_;

    // Gazebo variables
    gazebo::physics::ModelPtr track_model;
    gazebo::physics::LinkPtr car_link;
    ignition::math::Pose3d car_pos;

    // Parameters

    double lidar_total_view_distance;
    double camera_total_view_distance;
    double lidar_min_view_distance;
    double camera_min_view_distance;
    double lidar_x_view_distance;
    double lidar_y_view_distance;
    double lidar_fov;
    double camera_fov;
    double camera_a;
    double camera_b;
    bool lidar_on;

    double update_rate_;
    rclcpp::Time time_last_published;

    std::string cone_frame_;

    bool simulate_perception_;

    ignition::math::Vector3d perception_lidar_noise_;

    // Required ROS gazebo plugin variables
    gazebo::event::ConnectionPtr update_connection_;

    gazebo_ros::Node::SharedPtr rosnode_;

    unsigned int seed;

  };

} // namespace eufs
} // namespace gazebo_plugins

#endif //EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H
