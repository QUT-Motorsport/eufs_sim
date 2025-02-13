// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file was a copy of the ROS lidar with tiny modifactions.
// It has been update for the new version where we import the sensor
// and you can make modifications to the sensor in this file.
// At present, this file only renames the sensor with all the old
// code commented out.

// Header
#include <gazebo_lidar_plugin/gazebo_ros_lidar.hpp>

//Gazebo plugins
#include <gz/transport.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Lidar.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>

//ROS2 plugins

//C++ Includes
#include <memory>
#include <iostream>

namespace gazebo_plugins {

class GazeboRosLidarPrivate {
   public:

    // Aliases
    // using LaserScan = sensor_msgs::msg::LaserScan;
    // using PointCloud = sensor_msgs::msg::PointCloud;
    // using PointCloud2 = sensor_msgs::msg::PointCloud2;
    // using Range = sensor_msgs::msg::Range;
    // using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
    // using PointCloudPub = rclcpp::Publisher<PointCloud>::SharedPtr;
    // using PointCloud2Pub = rclcpp::Publisher<PointCloud2>::SharedPtr;
    // using RangePub = rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr;

    /// Pointer to the actual Lidar sensor
    std::shared_ptr<gz::sensors::Lidar> lidarSensor;

    // rclcpp::Node::SharedPtr rosNode; 

    /// Minimum intensity value to publish for laser scan / pointcloud messages
    double min_intensity_{0.0};

    std::string frameId = "lidar_frame";
};

GazeboRosLidar::GazeboRosLidar() = default;

GazeboRosLidar::~GazeboRosLidar() = default;

bool GazeboRosLidar::Load(const sdf::Sensor &sdfSensor) {
    // Create ros_node configured from sdf
    // impl_->ros_node_ =  gazebo_ros::Node::Get(_sdf);

    // rclcpp::QoS pub_qos(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // Get tf frame for output
    // impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

    if (!gz::sensors::Lidar::Load(sdfSensor))
    {
        gzerr << "[MyCustomLidar] Failed to load base LidarSensor.\n";
        return false;
    }

    // Get output type from sdf if provided
    // if (!_sdf->HasElement("output_type")) {
    //     RCLCPP_WARN(impl_->ros_node_->get_logger(), "missing <output_type>, defaults to sensor_msgs/PointCloud2");
    //     impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out", pub_qos);
    // } else {
    //     std::string output_type_string = _sdf->Get<std::string>("output_type");
    //     if (output_type_string == "sensor_msgs/LaserScan") {
    //         impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>("~/out", 10);
    //     } else if (output_type_string == "sensor_msgs/PointCloud") {
    //         impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud>("~/out", pub_qos);
    //     } else if (output_type_string == "sensor_msgs/PointCloud2") {
    //         impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out", pub_qos);
    //     } else if (output_type_string == "sensor_msgs/Range") {
    //         impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Range>("~/out", pub_qos);
    //     } else {
    //         RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Invalid <output_type> [%s]", output_type_string.c_str());
    //         return;
    //     }
    // }

    // if (!_sdf->HasElement("min_intensity")) {
    //     RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "missing <min_intensity>, defaults to %f", impl_->min_intensity_);
    // } else {
    //     impl_->min_intensity_ = _sdf->Get<double>("min_intensity");
    // }

    // Get parameters specific to Range output from sdf
    // if (impl_->pub_.type() == typeid(GazeboRosLidarPrivate::RangePub)) {
    //     if (!_sdf->HasElement("radiation_type")) {
    //         RCLCPP_INFO(impl_->ros_node_->get_logger(), "missing <radiation_type>, defaulting to infrared");
    //         impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    //     } else if ("ultrasound" == _sdf->Get<std::string>("radiation_type")) {
    //         impl_->range_radiation_type_ = sensor_msgs::msg::Range::ULTRASOUND;
    //     } else if ("infrared" == _sdf->Get<std::string>("radiation_type")) {
    //         impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    //     } else {
    //         RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Invalid <radiation_type> [%s]. Can be ultrasound or infrared",
    //                      _sdf->Get<std::string>("radiation_type").c_str());
    //         return;
    //     }
    // }
    
    // Create a ROS node directly
    // this->impl_->rosNode = rclcpp::Node::make_shared("gazebo_ros_lidar_node");

    // this->impl_->lidarSensor->SetUpdateCallback(
    // [this](const gz::sensors::LaserScan &_scan) -> void
    // {
        // Go wild in here
        // can call information like scans inside this.
    // }
    // );

    // Create gazebo transport node and subscribe to sensor's laser scan
    // impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
    // impl_->gazebo_node_->Init(_sensor->WorldName());

    // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
    // impl_->sensor_topic_ = _sensor->Topic();
    // impl_->SubscribeGazeboLaserScan();
    
    // this->SetUpdateCallback([this](const gz::sensors::LaserScan &scan)
    // {
    //     
    // });
    
    
    std::cout << "[GazeboRosLidar] Plugin loaded successfully." << std::endl;
    return true;
}

// void GazeboRosLidarPrivate::SubscribeGazeboLaserScan() {
//     if (pub_.type() == typeid(LaserScanPub)) {
//         laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_, &GazeboRosLidarPrivate::PublishLaserScan, this);
//     } else if (pub_.type() == typeid(PointCloudPub)) {
//         laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_, &GazeboRosLidarPrivate::PublishPointCloud, this);
//     } else if (pub_.type() == typeid(PointCloud2Pub)) {
//         laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_, &GazeboRosLidarPrivate::PublishPointCloud2, this);
//     } else if (pub_.type() == typeid(RangePub)) {
//         laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_, &GazeboRosLidarPrivate::PublishRange, this);
//     } else {
//         RCLCPP_ERROR(ros_node_->get_logger(), "Publisher is an invalid type. This is an internal bug.");
//     }
// }

// void GazeboRosLidarPrivate::PublishLaserScan(ConstLaserScanStampedPtr& _msg) {
//     // Convert Laser scan to ROS LaserScan
//     auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg);
//     // Set tf frame
//     ls.header.frame_id = frame_name_;
//     // Publish output
//     boost::get<LaserScanPub>(pub_)->publish(ls);
// }

// void GazeboRosLidarPrivate::PublishPointCloud(ConstLaserScanStampedPtr& _msg) {
//     // Convert Laser scan to PointCloud
//     auto pc = g s::Convert<sensor_msgs::msg::PointCloud>(*_msg, min_intensity_);
//     // Set tf frame
//     pc.header.frame_id = frame_name_;
//     // Publish output
//     boost::get<PointCloudPub>(pub_)->publish(pc);
// }

// void GazeboRosLidarPrivate::PublishPointCloud2(ConstLaserScanStampedPtr& _msg) {
//     // Convert Laser scan to PointCloud2
//     auto pc2 = gazebo_ros::Convert<sensor_msgs::msg::PointCloud2>(*_msg, min_intensity_);
//     // Set tf frame
//     pc2.header.frame_id = frame_name_;
//     // Publish output
//     boost::get<PointCloud2Pub>(pub_)->publish(pc2);
// }

// void GazeboRosLidarPrivate::PublishRange(ConstLaserScanStampedPtr& _msg) {
//     // Convert Laser scan to range
//     auto range_msg = gazebo_ros::Convert<sensor_msgs::msg::Range>(*_msg);
//     // Set tf frame
//     range_msg.header.frame_id = frame_name_;
//     // Set radiation type from sdf
//     range_msg.radiation_type = range_radiation_type_;
//     // Publish output
//     boost::get<RangePub>(pub_)->publish(range_msg);
// }

// Register this plugin with the simulator
// GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLidar)

}  // namespace gazebo_plugins