#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>

namespace gazebo_plugins {
namespace eufs_plugins {

class RaceCarPlugin : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
public:
  RaceCarPlugin();
  ~RaceCarPlugin() override;

  void Configure(
      const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override;

private:
  struct VehicleState {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double speed{0.0};
    double steering_rad{0.0};
  };

  void readSdfParams(const std::shared_ptr<const sdf::Element> &sdf);

  void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  bool resetVehiclePosition(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  bool resetCones(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void returnCommandMode(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void updateVehicle(double dt);
  void applyModelPose(gz::sim::EntityComponentManager &ecm);
  void publishOutputs(const gz::sim::UpdateInfo &info);
  void publishTf(const builtin_interfaces::msg::Time &stamp);

  nav_msgs::msg::Odometry makeOdomMsg(
      const builtin_interfaces::msg::Time &stamp,
      const std::string &frame_id,
      const std::string &child_frame_id) const;

  geometry_msgs::msg::TwistWithCovarianceStamped makeWheelTwistMsg(
      const builtin_interfaces::msg::Time &stamp) const;

  builtin_interfaces::msg::Time simTimeToRosTime(
      const std::chrono::steady_clock::duration &sim_time) const;

  double clamp(double value, double min_value, double max_value) const;

private:
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};

  std::shared_ptr<rclcpp::Node> ros_node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_cmd_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_velocity_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_angle_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_wheel_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_visual_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_slam_pose_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gt_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gt_velocity_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gt_steering_angle_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_gt_wheel_twist_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr command_mode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cones_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cones_system_service_;


  std::mutex command_mutex_;
  ackermann_msgs::msg::AckermannDriveStamped last_cmd_;
  bool has_command_{false};

  VehicleState state_;
  gz::math::Pose3d initial_pose_;

  double last_update_time_{-1.0};
  double last_publish_time_{-1.0};

  std::string command_mode_{"velocity"};

  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  bool publish_tf_{false};
  bool publish_ground_truth_{true};
  bool simulate_slam_{false};

  double wheelbase_{1.53};
  double max_speed_{20.0};
  double max_accel_{8.0};
  double max_decel_{12.0};
  double max_steering_deg_{35.0};
  double publish_rate_{50.0};
  double update_rate_{100.0};

  double command_timeout_{1.0};
  double last_command_sim_time_{-1.0};
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif