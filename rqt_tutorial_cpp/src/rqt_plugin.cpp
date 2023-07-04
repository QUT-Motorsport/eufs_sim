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

#include "rqt_tutorial_cpp/rqt_plugin.hpp"

#include <memory>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rqt_tutorial_cpp {
RQTPlugin::RQTPlugin() : rqt_gui_cpp::Plugin(), window_(0), rqt_node_(std::make_shared<rqt_tutorial_cpp::RQTNode>()) {
    setObjectName("RQT Tutorial CPP");
}

void RQTPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
    // Access standalone command line arguments
    QStringList argv = context.argv();
    // Create QWidget
    window_ = new QMainWindow();
    // Extend the widget with all attributes and children from UI file
    ui_.setupUi(window_);
    // Add widget to the user interface
    if (context.serialNumber() > 1) {
        window_->setWindowTitle(window_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(window_);

    // Set states
    connect(ui_.LV_key, SIGNAL(valueChanged(int)), this, SLOT(set_LV_key()));
    connect(ui_.TS_key, SIGNAL(valueChanged(int)), this, SLOT(set_TS_key()));
    connect(ui_.AS_key, SIGNAL(valueChanged(int)), this, SLOT(set_AS_key()));
    connect(ui_.SDC_btn, SIGNAL(clicked(bool)), this, SLOT(set_SDC_btn()));
    connect(ui_.TS_btn, SIGNAL(clicked(bool)), this, SLOT(set_TS_btn()));
    connect(ui_.mission_btn, SIGNAL(clicked(bool)), this, SLOT(set_mission_btn()));
    connect(ui_.mission_select, SIGNAL(currentIndexChanged(int)), this, SLOT(set_mission_dropdown()));
    connect(ui_.estop_btn, SIGNAL(toggled(bool)), this, SLOT(set_estop_btn()));
    connect(ui_.r2d_btn, SIGNAL(clicked(bool)), this, SLOT(set_r2d_btn()));
    connect(ui_.RES_safety, SIGNAL(valueChanged(int)), this, SLOT(set_switch_up()));

    // Run ros spin
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, SIGNAL(timeout()), this, SLOT(ros_timer_callback()));
    ros_timer_->start(10);
}

void RQTPlugin::shutdownPlugin() {
    // TODO(my_username): unregister all publishers here
}

void RQTPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
    // TODO(my_username): save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
    (void)plugin_settings;
    (void)instance_settings;
}

void RQTPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                const qt_gui_cpp::Settings& instance_settings) {
    // TODO(my_username): restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
    (void)plugin_settings;
    (void)instance_settings;
}

void RQTPlugin::ros_timer_callback() { rclcpp::spin_some(rqt_node_); }

void RQTPlugin::set_LV_key() {
    rqt_node_->LV_key_on = ui_.LV_key->value();
    std::cout << "LV_key_on: " << rqt_node_->LV_key_on << std::endl;
}

void RQTPlugin::set_TS_key() {
    rqt_node_->TS_key_on = ui_.TS_key->value();
    std::cout << "TS_key_on: " << rqt_node_->TS_key_on << std::endl;
}

void RQTPlugin::set_AS_key() {
    rqt_node_->AS_key_on = ui_.AS_key->value();
    std::cout << "AS_key_on: " << rqt_node_->AS_key_on << std::endl;
}

void RQTPlugin::set_SDC_btn() {
    rqt_node_->SDC_pressed = true;
    std::cout << "SDC_btn_on: " << rqt_node_->SDC_pressed << std::endl;
}

void RQTPlugin::set_TS_btn() {
    rqt_node_->TS_pressed = true;
    std::cout << "TS_btn_on: " << rqt_node_->TS_pressed << std::endl;
}

void RQTPlugin::set_mission_btn() {
    rqt_node_->mission_pressed = true;
    std::cout << "mission_btn_on: " << rqt_node_->mission_pressed << std::endl;
}

void RQTPlugin::set_mission_dropdown() {
    rqt_node_->selected_mission = ui_.mission_select->currentIndex();
    std::cout << "mission_select: " << rqt_node_->selected_mission << std::endl;
}

void RQTPlugin::set_estop_btn() {
    if (ui_.estop_btn->isChecked()) {
        rqt_node_->estopped = true;
    } else {
        rqt_node_->estopped = false;
    }
    std::cout << "estop_btn_on: " << rqt_node_->estopped << std::endl;
}

void RQTPlugin::set_r2d_btn() {
    rqt_node_->r2d_pressed = true;
    std::cout << "r2d_btn_on: " << rqt_node_->r2d_pressed << std::endl;
}

void RQTPlugin::set_switch_up() {
    rqt_node_->switch_up = ui_.RES_safety->value();
    std::cout << "RES_safety: " << rqt_node_->switch_up << std::endl;
}
}  // namespace rqt_tutorial_cpp
PLUGINLIB_EXPORT_CLASS(rqt_tutorial_cpp::RQTPlugin, rqt_gui_cpp::Plugin)
