#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class GzLidarRosBridge : public rclcpp::Node {
public:
  GzLidarRosBridge()
      : rclcpp::Node("gz_lidar_ros_bridge")
  {
    this->gz_topic_ = this->declare_parameter<std::string>("gz_topic", "/lidar");
    this->ros_topic_ = this->declare_parameter<std::string>("ros_topic", "/scan");
    this->frame_id_ = this->declare_parameter<std::string>("frame_id", "velodyne");
    this->scan_time_ = this->declare_parameter<double>("scan_time", 0.05);

    this->publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        this->ros_topic_,
        rclcpp::SensorDataQoS());

    const bool ok = this->gz_node_.Subscribe(
        this->gz_topic_,
        &GzLidarRosBridge::onScan,
        this);

    if (!ok) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to subscribe to Gazebo topic [%s]",
          this->gz_topic_.c_str());
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Bridging Gazebo [%s] -> ROS [%s], frame_id [%s]",
          this->gz_topic_.c_str(),
          this->ros_topic_.c_str(),
          this->frame_id_.c_str());
    }
  }

private:
  void onScan(const gz::msgs::LaserScan &_scan)
  {
    sensor_msgs::msg::LaserScan msg;

    if (_scan.header().has_stamp()) {
      msg.header.stamp.sec = static_cast<int32_t>(_scan.header().stamp().sec());
      msg.header.stamp.nanosec = static_cast<uint32_t>(_scan.header().stamp().nsec());
    } else {
      msg.header.stamp = this->now();
    }

    msg.header.frame_id = this->frame_id_;

    msg.angle_min = static_cast<float>(_scan.angle_min());
    msg.angle_max = static_cast<float>(_scan.angle_max());
    msg.angle_increment = static_cast<float>(_scan.angle_step());

    msg.time_increment = 0.0f;
    msg.scan_time = static_cast<float>(this->scan_time_);

    msg.range_min = static_cast<float>(_scan.range_min());
    msg.range_max = static_cast<float>(_scan.range_max());

    msg.ranges.reserve(_scan.ranges_size());
    for (int i = 0; i < _scan.ranges_size(); ++i) {
      msg.ranges.push_back(static_cast<float>(_scan.ranges(i)));
    }

    msg.intensities.reserve(_scan.intensities_size());
    for (int i = 0; i < _scan.intensities_size(); ++i) {
      msg.intensities.push_back(static_cast<float>(_scan.intensities(i)));
    }

    this->publisher_->publish(msg);
  }

private:
  std::string gz_topic_;
  std::string ros_topic_;
  std::string frame_id_;
  double scan_time_{0.05};

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GzLidarRosBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}