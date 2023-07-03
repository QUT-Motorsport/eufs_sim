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

#include <QStringList>
#include <memory>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rqt_tutorial_cpp/rqt_plugin.hpp"

namespace rqt_tutorial_cpp
{
RQTPlugin::RQTPlugin()
: rqt_gui_cpp::Plugin(),
  widget_(0),
  rqt_node_(std::make_shared<rqt_tutorial_cpp::RQTNode>())
{
  setObjectName("RQT Tutorial CPP");
}

void RQTPlugin::initPlugin(qt_gui_cpp::PluginContext & context)
{
  // Access standalone command line arguments
  QStringList argv = context.argv();
  // Create QWidget
  widget_ = new QWidget();
  // Extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // Add widget to the user interface
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Set states
  connect(ui_.pub_on_button, SIGNAL(clicked(bool)), this, SLOT(set_pub_on()));
  connect(ui_.pub_off_button, SIGNAL(clicked(bool)), this, SLOT(set_pub_off()));
  connect(ui_.sub_on_button, SIGNAL(clicked(bool)), this, SLOT(set_sub_on()));
  connect(ui_.sub_off_button, SIGNAL(clicked(bool)), this, SLOT(set_sub_off()));

  // Run ros spin
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, SIGNAL(timeout()), this, SLOT(ros_timer_callback()));
  ros_timer_->start(10);

  // Get & Display states
  display_timer_ = new QTimer(this);
  connect(display_timer_, SIGNAL(timeout()), this, SLOT(display_timer_callback()));
  display_timer_->start(10);
}

void RQTPlugin::shutdownPlugin()
{
  // TODO(my_username): unregister all publishers here
}

void RQTPlugin::saveSettings(
  qt_gui_cpp::Settings & plugin_settings,
  qt_gui_cpp::Settings & instance_settings) const
{
  // TODO(my_username): save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
  (void) plugin_settings;
  (void) instance_settings;
}

void RQTPlugin::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings,
  const qt_gui_cpp::Settings & instance_settings)
{
  // TODO(my_username): restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
  (void) plugin_settings;
  (void) instance_settings;
}

void RQTPlugin::ros_timer_callback()
{
  rclcpp::spin_some(rqt_node_);
}

void RQTPlugin::display_timer_callback()
{
  ui_.pub_onoff_state->setText(get_pub_onff());
  ui_.sub_onoff_state->setText(get_sub_onff());
}

QString RQTPlugin::get_pub_onff()
{
  QString q_str;
  if (rqt_node_->pub_onoff_ == true) {
    q_str = "on";
  } else {
    q_str = "off";
  }

  return q_str;
}

QString RQTPlugin::get_sub_onff()
{
  QString q_str;
  if (rqt_node_->sub_onoff_ == true) {
    q_str = "on";
  } else {
    q_str = "off";
  }

  return q_str;
}

void RQTPlugin::set_pub_on()
{
  rqt_node_->pub_onoff_ = true;
}

void RQTPlugin::set_pub_off()
{
  rqt_node_->pub_onoff_ = false;
}

void RQTPlugin::set_sub_on()
{
  rqt_node_->sub_onoff_ = true;
}

void RQTPlugin::set_sub_off()
{
  rqt_node_->sub_onoff_ = false;
}
}  // namespace rqt_tutorial_cpp
PLUGINLIB_EXPORT_CLASS(rqt_tutorial_cpp::RQTPlugin, rqt_gui_cpp::Plugin)
