#ifndef EUFS_PLUGINS_GAZEBO_SIMULATE_BOUNDING_BOXES_INCLUDE_GAZEBO_SIMULATE_BOUNDING_BOXES_GAZEBO_SIMULATE_BOUNDING_BOXES_HPP_
#define EUFS_PLUGINS_GAZEBO_SIMULATE_BOUNDING_BOXES_INCLUDE_GAZEBO_SIMULATE_BOUNDING_BOXES_GAZEBO_SIMULATE_BOUNDING_BOXES_HPP_

#include <string>
#include <vector>
#include <tuple>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// Gazebo header
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/buffer.h>

#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/bounding_boxes.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>

#include <image_geometry/pinhole_camera_model.h>

struct ConeInfo {
  double radius;
  double height;
};

struct BoundingBoxes {
  std::vector<std::vector<double>> blue_bounding_boxes;
  std::vector<std::vector<double>> yellow_bounding_boxes;
  std::vector<std::vector<double>> orange_bounding_boxes;
  std::vector<std::vector<double>> big_orange_bounding_boxes;
};

namespace gazebo_plugins {
namespace eufs_plugins {

class BoundingBoxesPlugin : public gazebo::ModelPlugin {
 public:
  BoundingBoxesPlugin();

  // Gazebo plugin functions
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

 private:
  void cones_callback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);

  void update_msg(eufs_msgs::msg::BoundingBoxes &bounding_boxes_msg, std_msgs::msg::Header header,
                  std::vector<std::vector<double>> blue_cones,
                  std::vector<std::vector<double>> yellow_cones,
                  std::vector<std::vector<double>> orange_cones,
                  std::vector<std::vector<double>> big_orange_cones);

  void getTransform(std::string target_frame_, std::string source_frame_);

  void setCameraInfo(rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
                      custom_cam_info_pub);

  std::default_random_engine generate_seed();

  void add_gaussian_noise_to_bounding_box(
              std::vector<double> &bounding_box_vector, std::default_random_engine seed);

  void push_back_bounding_boxes_msg(eufs_msgs::msg::BoundingBoxes &bounding_boxes_msg,
                                      std::vector<std::vector<double>> bounding_boxes_vector,
                                      std::string color) const;

  std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>>
      projectToImagePlane(std::vector<eufs_msgs::msg::ConeWithCovariance> &cones_vector,
                            const ConeInfo cone_info_);

  std::string getStringParameter(sdf::ElementPtr _sdf, const char *element,
                                    std::string default_value,
                                    const char *default_description);

  int getIntParameter(sdf::ElementPtr _sdf, const char *element,
                        int default_value,
                        const char *default_description);

  sensor_msgs::msg::CameraInfo cam_info_;

  image_geometry::PinholeCameraModel camera_model;

  struct BoundingBoxes noisy_bounding_boxes_container;

  // Gazebo variables
  gazebo::common::Time time_last_published;

  // Required ROS gazebo plugin variables
  gazebo::physics::WorldPtr _world;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr rosnode_;

  // Publisher
  rclcpp::Publisher<eufs_msgs::msg::BoundingBoxes>::SharedPtr ground_truth_bounding_boxes_pub;
  rclcpp::Publisher<eufs_msgs::msg::BoundingBoxes>::SharedPtr bounding_boxes_with_noise_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr custom_cam_info_pub;

  // Subscriber
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_ground_truth_sub_;

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform transform;

  // cone_info
  struct ConeInfo small_cones {0.11, 0.31};
  struct ConeInfo big_cones {0.13, 0.53};

  // Utility
  std::vector<int> bounding_box_vector;
  unsigned int seed;

  // Parameter declaration
  std::string target_frame_;
  std::string source_frame_;

  // Camera callibration
  int camera_width_;
  int camera_height_;
  std::vector<double> D;
  std::array<double, 9> K;
  std::array<double, 9> R;
  std::array<double, 12> P;
  std::string distortion_model_;

  // Gaussian Noise settings for Bounding Box ("bb")
  double mean_bb;
  double standard_deviation_bb;
  double mean_width;
  double standard_deviation_width;
  double mean_height;
  double standard_deviation_height;

  // Rate to publish ros messages
  double _publish_rate;
  gazebo::common::Time _last_sim_time;

  // Bounding Box Variables
  cv::Point2d top_left_point;
  cv::Point2d top_right_point;
  cv::Point2d bottom_left_point;
  cv::Point2d bottom_right_point;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_SIMULATE_BOUNDING_BOXES_INCLUDE_GAZEBO_SIMULATE_BOUNDING_BOXES_GAZEBO_SIMULATE_BOUNDING_BOXES_HPP_
