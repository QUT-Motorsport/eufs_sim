// Main Include
#include "gazebo_race_car_model/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

namespace gazebo_plugins
{

  GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)

  RaceCarModelPlugin::RaceCarModelPlugin() {}

  RaceCarModelPlugin::~RaceCarModelPlugin()
  {
    _updateConnection.reset();
  }

  void RaceCarModelPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    _rosnode = gazebo_ros::Node::Get(_sdf);

    RCLCPP_INFO(_rosnode->get_logger(), "Loading RaceCarModelPlugin");

    // _model = model;
    // _world = _model->GetWorld();

    // Initalize transform broadcaster
    // _tf_br = new tf2_ros::TransformBroadcaster(_rosnode);

    // Initialize state machine
    // _state_machine = new eufs::StateMachine(_rosnode);

    // TODO(angus): delete if everything is working fine
    // Initialize gazebo node (different node than a ROS node)
    // this->gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    // this->gznode->Init();
    // this->worldControlPub = this->gznode->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    // Initialize vehicle model
    // initVehicleModel(plugin_sdf);

    // Parse racecar params
    // initRacecarParam(plugin_sdf);

    // Obtain racecar model from Gazebo
    // initModel(plugin_sdf);

    // Store vehicle offset
    // setPositionFromWorld();

    // ROS Publishers
    // _pub_ground_truth_car_state = _rosnode->create_publisher<eufs_msgs::msg::CarState>(_ground_truth_car_state_topic, 1);
    // _pub_localisation_car_state = _rosnode->create_publisher<eufs_msgs::msg::CarState>(_localisation_car_state_topic, 1);
    // _pub_wheel_speeds = _rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(_wheel_speeds_topic_name, 1);
    // _pub_ground_truth_wheel_speeds = _rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(_ground_truth_wheel_speeds_topic_name, 1);
    // _pub_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>(_odom_topic_name, 1);

    // ROS Services
    // _reset_vehicle_pos_srv = _rosnode->create_service<std_srvs::srv::Trigger>("/ros_can/reset_vehicle_pos", std::bind(&RaceCarModelPlugin::resetVehiclePosition, this, std::placeholders::_1, std::placeholders::_2));

    // ROS Subscriptions
    // _sub_cmd = _rosnode->create_subscription<eufs_msgs::msg::AckermannDriveStamped>("/cmd", 1, std::bind(&RaceCarModelPlugin::onCmd, this, std::placeholders::_1));

    // Connect plugin to Gazebo update events
    _updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));
    // _lastSimTime = _world->SimTime();
    // _time_last_cmd = 0.0;

    RCLCPP_INFO(_rosnode->get_logger(), "RaceCarModelPlugin Loaded");
  }

  void RaceCarModelPlugin::initVehicleModel(sdf::ElementPtr &plugin_sdf)
  {
    // Get vehicle model name
    std::string vehicle_model;
    if (!plugin_sdf->HasElement("vehicle_model"))
    {
      RCLCPP_WARN(_rosnode->get_logger(), "No <vehicle_model> tag, defaulting to DynamicBicycle.");
      vehicle_model = "DynamicBicycle";
    }
    else
    {
      vehicle_model = plugin_sdf->GetElement("vehicle_model")->Get<std::string>();
    }

    // Get vehicle model config file
    std::string yaml_name;
    if (!plugin_sdf->HasElement("yaml_config"))
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "No <yaml_config> tag, cannot proceed");
      return;
    }
    else
    {
      yaml_name = plugin_sdf->GetElement("yaml_config")->Get<std::string>();
    }

    // Initialize vehicle model
    if (vehicle_model == "PointMass")
    {
      _vehicle = std::unique_ptr<eufs::VehicleModel>(new eufs::PointMass(yaml_name));
    }
    else if (vehicle_model == "DynamicBicycle")
    {
      _vehicle = std::unique_ptr<eufs::VehicleModel>(new eufs::DynamicBicycle(yaml_name));
    }
    else
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "Unknown vehicle model: %s", vehicle_model);
      return;
    }

    RCLCPP_DEBUG(_rosnode->get_logger(), "Initialized vehicle model");
  }

  void RaceCarModelPlugin::initRacecarParam(sdf::ElementPtr &plugin_sdf)
  {
    if (!plugin_sdf->HasElement("update_rate"))
    {
      _update_rate = 1000.0;
    }
    else
    {
      _update_rate = plugin_sdf->GetElement("update_rate")->Get<double>();
    }

    if (!plugin_sdf->HasElement("publish_rate"))
    {
      _publish_rate = 200.0;
    }
    else
    {
      _publish_rate = plugin_sdf->GetElement("publish_rate")->Get<double>();
    }

    if (!plugin_sdf->HasElement("referenceFrame"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <referenceFrame>, defaults to map");
      _reference_frame = "map";
    }
    else
    {
      _reference_frame = plugin_sdf->GetElement("referenceFrame")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("robotFrame"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <robotFrame>, defaults to base_footprint");
      _robot_frame = "base_footprint";
    }
    else
    {
      _robot_frame = plugin_sdf->GetElement("robotFrame")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("publishTransform"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <publishTransform>, defaults to false");
      _publish_tf = false;
    }
    else
    {
      _publish_tf = plugin_sdf->GetElement("publishTransform")->Get<bool>();
    }

    if (!plugin_sdf->HasElement("wheelSpeedsTopicName"))
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <wheelSpeedsTopicName>, cannot proceed");
      return;
    }
    else
    {
      _wheel_speeds_topic_name = plugin_sdf->GetElement("wheelSpeedsTopicName")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("groundTruthWheelSpeedsTopicName"))
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <groundTruthWheelSpeedsTopicName>, cannot proceed");
      return;
    }
    else
    {
      _ground_truth_wheel_speeds_topic_name = plugin_sdf->GetElement("groundTruthWheelSpeedsTopicName")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("groundTruthCarStateTopic"))
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <groundTruthCarStateTopic>, cannot proceed");
      return;
    }
    else
    {
      _ground_truth_car_state_topic = plugin_sdf->GetElement("groundTruthCarStateTopic")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("localisationCarStateTopic"))
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <localisationCarStateTopic>, cannot proceed");
      return;
    }
    else
    {
      _localisation_car_state_topic = plugin_sdf->GetElement("localisationCarStateTopic")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("odometryTopicName"))
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <odometryTopicName>, cannot proceed");
      return;
    }
    else
    {
      _odom_topic_name = plugin_sdf->GetElement("odometryTopicName")->Get<std::string>();
    }

    if (!plugin_sdf->HasElement("positionNoise"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <positionNoise>, defaults to 0.0, 0.0, 0.0");
      _position_noise = {0.0, 0.0, 0.0};
    }
    else
    {
      auto temp = plugin_sdf->GetElement("positionNoise")->Get<ignition::math::Vector3d>();
      _position_noise = {temp.X(), temp.Y(), temp.Z()};
    }

    if (_position_noise.size() != 3)
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "positionNoise parameter vector is not of size 3");
    }

    if (!plugin_sdf->HasElement("orientationNoise"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <orientationNoise>, defaults to 0.0, 0.0, 0.0");
      _orientation_noise = {0.0, 0.0, 0.0};
    }
    else
    {
      auto temp = plugin_sdf->GetElement("orientationNoise")->Get<ignition::math::Vector3d>();
      _orientation_noise = {temp.X(), temp.Y(), temp.Z()};
    }

    if (_orientation_noise.size() != 3)
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "orientationNoise parameter vector is not of size 3");
    }

    if (!plugin_sdf->HasElement("linearVelocityNoise"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <linearVelocityNoise>, defaults to 0.0, 0.0, 0.0");
      _linear_velocity_noise = {0.0, 0.0, 0.0};
    }
    else
    {
      auto temp = plugin_sdf->GetElement("linearVelocityNoise")->Get<ignition::math::Vector3d>();
      _linear_velocity_noise = {temp.X(), temp.Y(), temp.Z()};
    }

    if (_linear_velocity_noise.size() != 3)
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "linearVelocityNoise parameter vector is not of size 3");
    }

    if (!plugin_sdf->HasElement("angularVelocityNoise"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <angularVelocityNoise>, defaults to 0.0, 0.0, 0.0");
      _angular_velocity_noise = {0.0, 0.0, 0.0};
    }
    else
    {
      auto temp = plugin_sdf->GetElement("angularVelocityNoise")->Get<ignition::math::Vector3d>();
      _angular_velocity_noise = {temp.X(), temp.Y(), temp.Z()};
    }

    if (_angular_velocity_noise.size() != 3)
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "angularVelocityNoise parameter vector is not of size 3");
    }

    if (!plugin_sdf->HasElement("linearAccelerationNoise"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <linearAccelerationNoise>, defaults to 0.0, 0.0, 0.0");
      _linear_acceleration_noise = {0.0, 0.0, 0.0};
    }
    else
    {
      auto temp = plugin_sdf->GetElement("linearAccelerationNoise")->Get<ignition::math::Vector3d>();
      _linear_acceleration_noise = {temp.X(), temp.Y(), temp.Z()};
    }

    if (_linear_acceleration_noise.size() != 3)
    {
      RCLCPP_FATAL(_rosnode->get_logger(), "linearAccelerationNoise parameter vector is not of size 3");
    }

    if (!plugin_sdf->HasElement("commandMode"))
    {
      RCLCPP_DEBUG(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <commandMode>, defaults to acceleration");
      _command_mode = acceleration;
    }
    else
    {
      auto temp = plugin_sdf->GetElement("commandMode")->Get<std::string>();
      if (temp.compare("acceleration") == 0)
      {
        _command_mode = acceleration;
      }
      else if (temp.compare("velocity") == 0)
      {
        _command_mode = velocity;
      }
      else
      {
        RCLCPP_WARN(_rosnode->get_logger(), "commandMode parameter string is invalid, defaults to acceleration");
        _command_mode = acceleration;
      }
    }
  }

  void RaceCarModelPlugin::initModel(sdf::ElementPtr &plugin_sdf)
  {
    // Steering joints
    std::string leftSteeringJointName = _model->GetName() + "::" + plugin_sdf->Get<std::string>("front_left_wheel_steering");
    _left_steering_joint = _model->GetJoint(leftSteeringJointName);
    std::string rightSteeringJointName = _model->GetName() + "::" + plugin_sdf->Get<std::string>("front_right_wheel_steering");
    _right_steering_joint = _model->GetJoint(rightSteeringJointName);

    // Front wheels
    std::string frontLeftWheelName = _model->GetName() + "::" + plugin_sdf->Get<std::string>("front_left_wheel");
    _front_left_wheel = _model->GetJoint(frontLeftWheelName);
    std::string frontRightWheelName = _model->GetName() + "::" + plugin_sdf->Get<std::string>("front_right_wheel");
    _front_right_wheel = _model->GetJoint(frontRightWheelName);

    // Rear wheels
    std::string rearLeftWheelName = _model->GetName() + "::" + plugin_sdf->Get<std::string>("rear_left_wheel");
    _rear_left_wheel = _model->GetJoint(rearLeftWheelName);
    std::string rearRightWheelName = _model->GetName() + "::" + plugin_sdf->Get<std::string>("rear_right_wheel");
    _rear_right_wheel = _model->GetJoint(rearRightWheelName);
  }

  void RaceCarModelPlugin::Reset()
  {
    _lastSimTime = 0;
  }

  void RaceCarModelPlugin::update()
  {
    // gazebo::common::Time curTime = _world->SimTime();
    // double dt = (curTime - _lastSimTime).Double();

    // if (dt < (1 / _update_rate))
    // {
    //   return;
    // }

    // _lastSimTime = curTime;
    // updateVehicle(dt, curTime);
  }

  void RaceCarModelPlugin::updateVehicle(const double dt, gazebo::common::Time current_time)
  {
    if (_command_mode == velocity)
    {
      double current_speed = std::sqrt(std::pow(_state.v_x, 2) + std::pow(_state.v_y, 2));
      _input.acc = (_input.vel - current_speed) / dt;
    }
    _input.acc = _rosnode->now().seconds() - _time_last_cmd < 1.0 ? _input.acc : -1.0;

    _left_steering_joint->SetPosition(0, _input.delta);
    _right_steering_joint->SetPosition(0, _input.delta);

    eufs::State new_state = _state;
    eufs::Input new_input = _input;

    _vehicle->updateState(new_state, new_input, dt);

    _state = new_state;
    _input = new_input;

    setModelState();

    double time_since_last_published = (current_time - _time_last_published).Double();

    if (time_since_last_published < (1 / _publish_rate))
    {
      return;
    }

    _time_last_published = current_time;

    // Publish Everything
    publishCarState();
    publishWheelSpeeds();
    publishOdom();

    if (_publish_tf)
    {
      publishTf();
    }

    _state_machine->spinOnce(current_time);
  }

  void RaceCarModelPlugin::setModelState()
  {
    double yaw = _state.yaw + _offset.Rot().Yaw();

    double x = _offset.Pos().X() + _state.x * cos(_offset.Rot().Yaw()) - _state.y * sin(_offset.Rot().Yaw());
    double y = _offset.Pos().Y() + _state.x * sin(_offset.Rot().Yaw()) + _state.y * cos(_offset.Rot().Yaw());
    double z = _model->WorldPose().Pos().Z();

    double vx = _state.v_x * cos(yaw) - _state.v_y * sin(yaw);
    double vy = _state.v_x * sin(yaw) + _state.v_y * cos(yaw);

    const ignition::math::Pose3d pose(x, y, z, 0, 0.0, yaw);
    const ignition::math::Vector3d vel(vx, vy, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, _state.r);

    _model->SetWorldPose(pose);
    _model->SetAngularVel(angular);
    _model->SetLinearVel(vel);
  }

  void RaceCarModelPlugin::setPositionFromWorld()
  {
    _offset = _model->WorldPose();

    RCLCPP_DEBUG(_rosnode->get_logger(),
                 "Got starting offset %f %f %f",
                 _offset.Pos()[0],
                 _offset.Pos()[1],
                 _offset.Pos()[2]);

    _state.x = 0.0;
    _state.y = 0.0;
    _state.yaw = 0.0;
    _state.v_x = 0.0;
    _state.v_y = 0.0;
    _state.r = 0.0;
    _state.a_x = 0.0;
    _state.a_y = 0.0;
  }

  bool RaceCarModelPlugin::resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // suppress unused parameter warning
    (void)response; // suppress unused parameter warning

    _state.x = 0.0;
    _state.y = 0.0;
    _state.yaw = 0.0;
    _state.v_x = 0.0;
    _state.v_y = 0.0;
    _state.r = 0.0;
    _state.a_x = 0.0;
    _state.a_y = 0.0;

    const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

    _model->SetWorldPose(_offset);
    _model->SetAngularVel(angular);
    _model->SetLinearVel(vel);

    return response->success;
  }

  void RaceCarModelPlugin::publishCarState()
  {
    // Publish Car Info
    eufs_msgs::msg::CarState car_state;

    car_state.header.stamp = _rosnode->now();
    car_state.header.frame_id = _reference_frame;
    car_state.child_frame_id = _robot_frame;

    double z = _model->WorldPose().Pos().Z();

    car_state.pose.pose.position.x = _state.x;
    car_state.pose.pose.position.y = _state.y;
    car_state.pose.pose.position.z = z;

    std::vector<double> orientation = {_state.yaw, 0.0, 0.0};

    orientation = this->ToQuaternion(orientation);

    car_state.pose.pose.orientation.x = orientation[0];
    car_state.pose.pose.orientation.y = orientation[1];
    car_state.pose.pose.orientation.z = orientation[2];
    car_state.pose.pose.orientation.w = orientation[3];

    car_state.twist.twist.linear.x = _state.v_x;
    car_state.twist.twist.linear.y = _state.v_y;
    car_state.twist.twist.linear.z = 0;
    car_state.twist.twist.angular.x = 0;
    car_state.twist.twist.angular.y = 0;
    car_state.twist.twist.angular.z = _state.r;

    car_state.linear_acceleration.x = _state.a_x;
    car_state.linear_acceleration.y = _state.a_y;
    car_state.linear_acceleration.z = 0;

    // TODO(angus): figure out this problem
    // car_state.slip_angle = _state.slip_angle;

    // TODO: make this more realistic
    car_state.state_of_charge = 999;

    // Publish ground_truth
    if (_pub_ground_truth_car_state->get_subscription_count() > 0)
    {
      _pub_ground_truth_car_state->publish(car_state);
    }

    // Add noise
    car_state.pose.pose.position.x += GaussianKernel(0, _position_noise[0]);
    car_state.pose.pose.position.y += GaussianKernel(0, _position_noise[1]);
    car_state.pose.pose.position.z += GaussianKernel(0, _position_noise[2]);

    // Reset orientation
    orientation = {_state.yaw, 0.0, 0.0};

    orientation[0] += GaussianKernel(0, _orientation_noise[0]);
    orientation[1] += GaussianKernel(0, _orientation_noise[1]);
    orientation[2] += GaussianKernel(0, _orientation_noise[2]);

    orientation = ToQuaternion(orientation);

    car_state.pose.pose.orientation.x = orientation[0];
    car_state.pose.pose.orientation.y = orientation[1];
    car_state.pose.pose.orientation.z = orientation[2];
    car_state.pose.pose.orientation.w = orientation[3];

    car_state.twist.twist.linear.x += GaussianKernel(0, _linear_velocity_noise[0]);
    car_state.twist.twist.linear.y += GaussianKernel(0, _linear_velocity_noise[1]);
    car_state.twist.twist.linear.z += GaussianKernel(0, _linear_velocity_noise[2]);
    car_state.twist.twist.angular.x += GaussianKernel(0, _angular_velocity_noise[0]);
    car_state.twist.twist.angular.y += GaussianKernel(0, _angular_velocity_noise[1]);
    car_state.twist.twist.angular.z += GaussianKernel(0, _angular_velocity_noise[2]);

    // Fill in covariance matrix
    car_state.pose.covariance[0] = pow(_position_noise[0], 2);
    car_state.pose.covariance[7] = pow(_position_noise[1], 2);
    car_state.pose.covariance[14] = pow(_position_noise[2], 2);
    car_state.pose.covariance[21] = pow(_orientation_noise[0], 2);
    car_state.pose.covariance[28] = pow(_orientation_noise[1], 2);
    car_state.pose.covariance[35] = pow(_orientation_noise[2], 2);

    car_state.twist.covariance[0] = pow(_linear_velocity_noise[0], 2);
    car_state.twist.covariance[7] = pow(_linear_velocity_noise[1], 2);
    car_state.twist.covariance[14] = pow(_linear_velocity_noise[2], 2);
    car_state.twist.covariance[21] = pow(_angular_velocity_noise[0], 2);
    car_state.twist.covariance[28] = pow(_angular_velocity_noise[1], 2);
    car_state.twist.covariance[35] = pow(_angular_velocity_noise[2], 2);

    car_state.linear_acceleration.x += GaussianKernel(0, _linear_acceleration_noise[0]);
    car_state.linear_acceleration.y += GaussianKernel(0, _linear_acceleration_noise[1]);
    car_state.linear_acceleration.z += GaussianKernel(0, _linear_acceleration_noise[2]);

    car_state.linear_acceleration_covariance[0] = pow(_linear_acceleration_noise[0], 2);
    car_state.linear_acceleration_covariance[4] = pow(_linear_acceleration_noise[1], 2);
    car_state.linear_acceleration_covariance[8] = pow(_linear_acceleration_noise[2], 2);

    // Publish with noise
    if (_pub_localisation_car_state->get_subscription_count() > 0)
    {
      _pub_localisation_car_state->publish(car_state);
    }
  }

  void RaceCarModelPlugin::publishWheelSpeeds()
  {
    eufs_msgs::msg::WheelSpeedsStamped wheel_speeds;

    wheel_speeds.header.stamp = _rosnode->now();
    wheel_speeds.header.frame_id = _robot_frame;

    wheel_speeds.steering = _input.delta;

    wheel_speeds.lf_speed = 999;
    wheel_speeds.rf_speed = 999;

    float PI = 3.14159265;
    float wheel_circumference = 2 * PI * _vehicle->getParam().tire.radius;

    // Calculate Wheel speeds
    wheel_speeds.lb_speed = (_state.v_x / wheel_circumference) * 60;
    wheel_speeds.rb_speed = (_state.v_x / wheel_circumference) * 60;

    // Publish ground truth
    if (_pub_ground_truth_wheel_speeds->get_subscription_count() > 0)
    {
      _pub_ground_truth_wheel_speeds->publish(wheel_speeds);
    }

    // TODO: Add Noise to Wheel speeds here

    // Publish with Noise
    if (_pub_wheel_speeds->get_subscription_count() > 0)
    {
      _pub_wheel_speeds->publish(wheel_speeds);
    }
  }

  void RaceCarModelPlugin::publishOdom()
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = _rosnode->now();

    odom.header.frame_id = _reference_frame;
    odom.child_frame_id = _robot_frame;

    double z = _model->WorldPose().Pos().Z();

    odom.pose.pose.position.x = _state.x + GaussianKernel(0, _position_noise[0]);
    odom.pose.pose.position.y = _state.y + GaussianKernel(0, _position_noise[1]);
    odom.pose.pose.position.z = z + GaussianKernel(0, _position_noise[2]);

    std::vector<double> orientation = {_state.yaw, 0.0, 0.0};

    orientation[0] += GaussianKernel(0, _orientation_noise[0]);
    orientation[1] += GaussianKernel(0, _orientation_noise[1]);
    orientation[2] += GaussianKernel(0, _orientation_noise[2]);

    orientation = ToQuaternion(orientation);

    odom.pose.pose.orientation.x = orientation[0];
    odom.pose.pose.orientation.y = orientation[1];
    odom.pose.pose.orientation.z = orientation[2];
    odom.pose.pose.orientation.w = orientation[3];

    odom.twist.twist.linear.x = _state.v_x + GaussianKernel(0, _linear_velocity_noise[0]);
    odom.twist.twist.linear.y = _state.v_y + GaussianKernel(0, _linear_velocity_noise[1]);
    odom.twist.twist.linear.z = 0 + GaussianKernel(0, _linear_velocity_noise[2]);
    odom.twist.twist.angular.x = 0 + GaussianKernel(0, _angular_velocity_noise[0]);
    odom.twist.twist.angular.y = 0 + GaussianKernel(0, _angular_velocity_noise[1]);
    odom.twist.twist.angular.z = _state.r + GaussianKernel(0, _angular_velocity_noise[2]);

    // fill in covariance matrix
    odom.pose.covariance[0] = pow(_position_noise[0], 2);
    odom.pose.covariance[7] = pow(_position_noise[1], 2);
    odom.pose.covariance[14] = pow(_position_noise[2], 2);
    odom.pose.covariance[21] = pow(_orientation_noise[0], 2);
    odom.pose.covariance[28] = pow(_orientation_noise[1], 2);
    odom.pose.covariance[35] = pow(_orientation_noise[2], 2);

    odom.twist.covariance[0] = pow(_linear_velocity_noise[0], 2);
    odom.twist.covariance[7] = pow(_linear_velocity_noise[1], 2);
    odom.twist.covariance[14] = pow(_linear_velocity_noise[2], 2);
    odom.twist.covariance[21] = pow(_angular_velocity_noise[0], 2);
    odom.twist.covariance[28] = pow(_angular_velocity_noise[1], 2);
    odom.twist.covariance[35] = pow(_angular_velocity_noise[2], 2);

    if (_pub_odom->get_subscription_count() > 0)
    {
      _pub_odom->publish(odom);
    }
  }

  void RaceCarModelPlugin::publishTf()
  {
    // Position
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(_state.x + GaussianKernel(0, _position_noise[0]),
                                     _state.y + GaussianKernel(0, _position_noise[1]),
                                     0.0));

    // Orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, _state.yaw + GaussianKernel(0, _angular_velocity_noise[2]));
    transform.setRotation(q);

    // Send TF
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = _rosnode->now();
    transform_stamped.header.frame_id = _reference_frame;
    transform_stamped.child_frame_id = _robot_frame;
    tf2::convert(transform, transform_stamped.transform);

    _tf_br->sendTransform(transform_stamped);
  }

  void RaceCarModelPlugin::onCmd(const eufs_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    // TODO: Should add delay to the controls
    if (_state_machine->canDrive())
    {
      _input.delta = msg->drive.steering_angle;
      _input.acc = msg->drive.acceleration;
      _input.vel = msg->drive.speed;
    }
    else
    {
      // TODO: Should do something else to stop the car but is this good for now
      _input.delta = 0;
      _input.acc = -100;
      _input.vel = 0;
    }

    _time_last_cmd = _rosnode->now().seconds();
  }

  std::vector<double> RaceCarModelPlugin::ToQuaternion(std::vector<double> &euler)
  {
    // Abbreviations for the various angular functions
    double cy = cos(euler[0] * 0.5);
    double sy = sin(euler[0] * 0.5);
    double cp = cos(euler[1] * 0.5);
    double sp = sin(euler[1] * 0.5);
    double cr = cos(euler[2] * 0.5);
    double sr = sin(euler[2] * 0.5);

    std::vector<double> q;
    q.reserve(4);
    q[0] = cy * cp * sr - sy * sp * cr; // x
    q[1] = sy * cp * sr + cy * sp * cr; // y
    q[2] = sy * cp * cr - cy * sp * sr; // z
    q[3] = cy * cp * cr + sy * sp * sr; // w

    return q;
  }

} // namespace gazebo_plugins