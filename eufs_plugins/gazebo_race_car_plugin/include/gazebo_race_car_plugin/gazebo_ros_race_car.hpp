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

#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_

// Standard C++ Includes
#include <memory>
#include <string>
#include <vector>

// ROS Includes
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

// Gazebo Includes
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/EventManager.hh>
#include <gz/transport/Node.hh>

// Local includes
#include "eufs_models/eufs_models.hpp"
#include "helpers_ros.hpp"


namespace gazebo_plugins {
namespace eufs_plugins {

class RaceCarPlugin : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
   public:
   // Constuctor + Deconstructor
    RaceCarPlugin();
    ~RaceCarPlugin();

    //-----------------------------------------------------------------------------
    // Required Gazebo Plugin Interface Methods
    //-----------------------------------------------------------------------------
    /**
    * @brief Called once when the plugin is loaded. Formerly Load().
    * @param[in] entity The Entity handle to the model in the world.
    * @param[in] sdf    SDF configuration for this plugin.
    * @param[in] ecm    The EntityComponentManager to manipulate components.
    * @param[in] eventMgr Unused here, can be used for event connections.
    */

    void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

    /**
    * @brief Called before each iteration of the simulation (post-update).
    * @param[in] info Information about the current simulation step.
    * @param[in] ecm  The EntityComponentManager for reading/updating components.
    */
    void PreUpdate(const gz::sim::UpdateInfo &info,
                    gz::sim::EntityComponentManager &ecm) override;

    //-----------------------------------------------------------------------------
    // Internal Methods
    //-----------------------------------------------------------------------------
    /// Initialize ROS parameters, vehicle model, noise, etc.
    void initParams();

    /// Set initial position from SDF (or zero) into _offset, reset internal state
    void setPositionFromWorld();

    /// Reset the vehicle position/service callback
    bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response>);
    
    /// Return current command mode ("acceleration" or "velocity")
    void returnCommandMode(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response>);
    
    /// Send pose/velocity commands via ECM (PoseCmd, LinearVelocityCmd, etc.)
    void setModelState(gz::sim::EntityComponentManager &ecm);

    /// Convert odometry message to pose-with-covariance
    geometry_msgs::msg::PoseWithCovarianceStamped
        odomToPoseMsg(const nav_msgs::msg::Odometry &odom_msg);
    
    /// Compute wheel twist from 4 wheel speeds + steering angle
    geometry_msgs::msg::TwistWithCovarianceStamped
        getWheelTwist(const std::vector<double> &speeds, const double &angle);
    
    /// Convert internal eufs::models::State to nav_msgs::msg::Odometry
    nav_msgs::msg::Odometry stateToOdom(const eufs::models::State &state);

    /// Make a "visual odom" version of the above
    nav_msgs::msg::Odometry getVisualOdom(const nav_msgs::msg::Odometry &odom);

    /// Publish odometry, wheel speeds, velocity, etc. to ROS
    void publishVehicleMotion();

    /// Publish TF transforms (map->odom, odom->base_link) if enabled
    void publishTf();

    /**
    * @brief Main per-iteration logic called from PostUpdate. It calculates dt,
    *        updates the vehicle model, sets the new pose in ECM, and triggers
    *        publishVehicleMotion() at the desired rate.
    * @param[in] ecm  The entity-component manager for reading/updating components
    * @param[in] info The simulation step info (time, paused, etc.)
    * @param[in] dt   Computed delta-time for this iteration
    */
    void update(gz::sim::EntityComponentManager &ecm,
                const gz::sim::UpdateInfo &info,
                double dt);

    /// Apply the current steering angle to the steering joints
    void applySteeringJointPositions(gz::sim::EntityComponentManager &ecm);

    /// Handle AckermannDrive commands from ROS
    void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

private:
    //-----------------------------------------------------------------------------
    // Data Members
    //-----------------------------------------------------------------------------
    /// Plugin entity references
    gz::sim::Entity _entity{gz::sim::kNullEntity};
    gz::sim::Model  _model{gz::sim::kNullEntity};

    /// Joints for steering
    gz::sim::Entity _left_steering_joint{gz::sim::kNullEntity};
    gz::sim::Entity _right_steering_joint{gz::sim::kNullEntity};

    /// Drive wheel joints
    gz::sim::Entity _left_rear_wheel_joint{gz::sim::kNullEntity};
    gz::sim::Entity _right_rear_wheel_joint{gz::sim::kNullEntity};

    /// Node for ROS interaction
    std::shared_ptr<rclcpp::Node> _rosnode;

    // TF Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_br;

    /// eufs vehicle model + noise models
    std::unique_ptr<eufs::models::VehicleModel> _vehicle;
    std::unique_ptr<eufs::models::Noise> _noise;

    /// Internal state, offsets, etc.
    eufs::models::State _state;
    eufs::models::Input _des_input, _act_input;
    gz::math::Pose3d _offset; ///< Pose offset from SDF or zero
    
    // Former references
    // gz::transport::Node _transport_node;
    // std::chrono::steady_clock::time_point _last_sim_time, _last_cmd_time;

    /// For rate-limiting updates and publishes
    double _update_rate{0.0};
    double _publish_rate{0.0};
    double _last_sim_time{0.0};
    double _time_last_published{0.0};
    double _time_last_vis_odom_published{0.0};
    double _time_last_sbg_odom_published{0.0};

    /// Steering rate limit, etc.
    double _steering_lock_time{1.0};
    double _max_steering_rate{0.0};

    /// Command mode for the plugin
    enum CommandMode { acceleration, velocity };
    CommandMode _command_mode{acceleration};

    /// If true, publish ground-truth data
    bool _pub_gt{false};
    /// If true, simulate SLAM data
    bool _simulate_slam{false};
    /// If true, broadcast TF frames
    bool _pub_tf{false};

    /// ROS frames
    std::string _map_frame{"map"};
    std::string _odom_frame{"odom"};
    std::string _base_frame{"base_link"};

    /// Time-based control delay
    double _control_delay{0.5};

    /// Last command message
    ackermann_msgs::msg::AckermannDriveStamped _last_cmd;
    double _last_cmd_time{0.0};

    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _pub_wheel_twist;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_vis_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pub_pose;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_velocity;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_joint_state;

    // Ground truth publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _pub_gt_wheel_twist;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_gt_odom;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_gt_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_gt_velocity;

    /// Subscribers
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_cmd;

    /// Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_vehicle_pos_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _command_mode_srv;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
