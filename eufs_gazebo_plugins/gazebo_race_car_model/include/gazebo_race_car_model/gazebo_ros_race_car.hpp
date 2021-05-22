#ifndef GAZEBO_ROS_RACE_CAR_HPP
#define GAZEBO_ROS_RACE_CAR_HPP

// ROS Includes
#include "rclcpp/rclcpp.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"

// ROS messages
#include "eufs_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/vector3.hpp"

// ROS srvs
#include <std_srvs/srv/trigger.hpp>

// Gazebo Includes
#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo_ros/node.hpp"

// eufs_model vehicle models
#include "eufs_models/vehicle_model.hpp"
#include "eufs_models/dynamic_bicycle.hpp"
#include "eufs_models/point_mass.hpp"

// eufs racecar state machine
#include "state_machine.hpp"

namespace gazebo_plugins
{

  class RaceCarModelPlugin : public gazebo::ModelPlugin
  {
  public:
    RaceCarModelPlugin();

    ~RaceCarModelPlugin();

    void Reset();

    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // TODO(angus): delete if everything is working fine
    // gazebo::transport::NodePtr gznode;

    void update();
    void updateVehicle(double dt, gazebo::common::Time current_time);

    void initModel(sdf::ElementPtr &plugin_sdf);
    void initRacecarParam(sdf::ElementPtr &plugin_sdf);
    void initVehicleModel(sdf::ElementPtr &plugin_sdf);
    void setPositionFromWorld();
    bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setModelState();
    void publishCarState();
    void publishWheelSpeeds();
    void publishOdom();
    void publishTf();
    void onCmd(const eufs_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    eufs::State &getState() { return _state; }
    eufs::Input &getInput() { return _input; }

    ignition::math::Pose3d _offset;
    double _update_rate;
    gazebo::physics::WorldPtr _world;

    gazebo::event::ConnectionPtr _updateConnection;
    gazebo::common::Time _lastSimTime;
    gazebo::transport::PublisherPtr _worldControlPub;

    // States
    eufs::StateMachine *_state_machine;
    eufs::State _state;
    eufs::Input _input;
    double _time_last_cmd;

    // Pointer to the vehicle model
    eufs::VehicleModelPtr _vehicle;

    // Pointer to the parent model
    gazebo::physics::ModelPtr _model;

    // ROS Node
    gazebo_ros::Node::SharedPtr _rosnode;

    // ROS TF
    tf2_ros::TransformBroadcaster *_tf_br;

    // Rate to publish ROS messages
    double _publish_rate;
    gazebo::common::Time _time_last_published;

    // ROS parameters
    std::string _ground_truth_car_state_topic;
    std::string _localisation_car_state_topic;
    std::string _wheel_speeds_topic_name;
    std::string _ground_truth_wheel_speeds_topic_name;
    std::string _odom_topic_name;

    // ROS Publishers
    rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr _pub_ground_truth_car_state;
    rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr _pub_localisation_car_state;
    rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr _pub_wheel_speeds;
    rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr _pub_ground_truth_wheel_speeds;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;

    // ROS Subscriptions
    rclcpp::Subscription<eufs_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_cmd;

    // ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_vehicle_pos_srv; /// Service to reset vehicle position

    // Racecar parameters
    bool _publish_tf;
    std::string _reference_frame;
    std::string _robot_frame;

    std::vector<double> _position_noise;
    std::vector<double> _orientation_noise;
    std::vector<double> _linear_velocity_noise;
    std::vector<double> _angular_velocity_noise;
    std::vector<double> _linear_acceleration_noise;

    // Steering jointsState
    gazebo::physics::JointPtr _left_steering_joint;
    gazebo::physics::JointPtr _right_steering_joint;

    // Wheels
    gazebo::physics::JointPtr _front_left_wheel;
    gazebo::physics::JointPtr _front_right_wheel;
    gazebo::physics::JointPtr _rear_left_wheel;
    gazebo::physics::JointPtr _rear_right_wheel;

    enum CommandMode
    {
      acceleration,
      velocity
    };
    CommandMode _command_mode;

    /// @brief Converts an euler orientation to quaternion
    std::vector<double> ToQuaternion(std::vector<double> &euler);

    double GaussianKernel(double mu, double sigma)
    {
      // using Box-Muller transform to generate two independent standard
      // normally distributed normal variables see wikipedia
      unsigned seed = 0;
      // normalized uniform random variable
      double U = static_cast<double>(rand_r(&seed)) /
                 static_cast<double>(RAND_MAX);

      // normalized uniform random variable
      double V = static_cast<double>(rand_r(&seed)) /
                 static_cast<double>(RAND_MAX);

      double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
      // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

      // there are 2 indep. vars, we'll just use X
      // scale to our mu and sigma
      X = sigma * X + mu;
      return X;
    }
  };

} // namespace gazebo_plugins

#endif // GAZEBO_ROS_RACE_CAR_HPP
