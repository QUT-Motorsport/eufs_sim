// Copyright 2021 Jaehyun Shim
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

#ifndef RQT_TUTORIAL_CPP__RQT_NODE_HPP_
#define RQT_TUTORIAL_CPP__RQT_NODE_HPP_

#include <QStringListModel>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace rqt_tutorial_cpp
{
class RQTNode : public rclcpp::Node
{
public:
  RQTNode();
  virtual ~RQTNode();

  bool pub_onoff_ = true;
  bool sub_onoff_ = false;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void chatter_callback(const std_msgs::msg::String::SharedPtr msg);
  void timer_callback();
};
}  // namespace rqt_tutorial_cpp
#endif  // RQT_TUTORIAL_CPP__RQT_NODE_HPP_
