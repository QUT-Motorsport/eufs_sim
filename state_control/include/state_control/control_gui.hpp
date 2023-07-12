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

#ifndef STATE_CONTROL__CONTROL_GUI_HPP_
#define STATE_CONTROL__CONTROL_GUI_HPP_

#include <rqt_gui_cpp/plugin.h>
#include <ui_control_gui.h>

#include <QStringList>
#include <QTimer>
#include <QWidget>
#include <memory>

#include "state_control/state_node.hpp"

namespace state_control {
class ControlGUIPlugin : public rqt_gui_cpp::Plugin {
    Q_OBJECT

   public:
    ControlGUIPlugin();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings);

   private:
    Ui::MainWindow ui_;
    QMainWindow* window_;
    std::shared_ptr<state_control::StateNode> state_node_;
    QTimer* ros_timer_;

   private slots:
    void ros_timer_callback();

    void set_LV_key();
    void set_TS_key();
    void set_AS_key();
    void set_SDC_btn();
    void set_TS_btn();
    void set_mission_btn();
    void set_mission_dropdown();
    void set_estop_btn();
    void set_r2d_btn();
    void set_switch_up();
    void set_reset_btn();
};
}  // namespace state_control
#endif  // STATE_CONTROL__CONTROL_GUI_HPP_
