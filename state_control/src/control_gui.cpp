#include "state_control/control_gui.hpp"

#include <memory>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace state_control {
ControlGUIPlugin::ControlGUIPlugin()
    : rqt_gui_cpp::Plugin(), window_(0), state_node_(std::make_shared<state_control::StateNode>()) {
    setObjectName("State Control GUI");
}

void ControlGUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
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
    connect(ui_.reset_btn, SIGNAL(clicked(bool)), this, SLOT(set_reset_btn()));
    connect(ui_.laps_box, SIGNAL(valueChanged(double)), this, SLOT(set_laps()));

    // Run ros spin
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, SIGNAL(timeout()), this, SLOT(ros_timer_callback()));
    ros_timer_->start(10);
}

void ControlGUIPlugin::ros_timer_callback() {
    rclcpp::spin_some(state_node_);
    Car_State_t sim_car_state = state_node_->get_car_state();
    driverless_msgs::msg::State supervisor_state = state_node_->get_ros_state();

    // convert enum to string
    std::string veh_state_str = AS_STATES_STR[sim_car_state.AS_state];
    std::string TS_state_str = TS_STATES_STR[sim_car_state.TS_state];

    // stringify driverless_msgs::msg::State enum
    std::string supervisor_mission_str = ROS_MISSIONS_STR[supervisor_state.mission];
    std::string supervisor_state_str = stringify_state(supervisor_state);

    // update gui
    ui_.VehStateDisplay->setText(QString::fromStdString(veh_state_str));
    ui_.TSStateDisplay->setText(QString::fromStdString(TS_state_str));
    ui_.MissionDisplay->setText(QString::fromStdString(supervisor_mission_str));
    ui_.ASStateDisplay->setText(QString::fromStdString(supervisor_state_str));
    ui_.LapDisplay->setText(QString::number(supervisor_state.lap_count));
}

std::string ControlGUIPlugin::stringify_state(driverless_msgs::msg::State msg) {
    std::string state_str = "";

    switch (msg.state) {
        case driverless_msgs::msg::State::START:
            state_str = "START";
            break;
        case driverless_msgs::msg::State::SELECT_MISSION:
            state_str = "SELECT_MISSION";
            break;
        case driverless_msgs::msg::State::WAIT_FOR_MISSION:
            state_str = "WAIT_FOR_MISSION";
            break;
        case driverless_msgs::msg::State::CHECK_EBS:
            state_str = "CHECK_EBS";
            break;
        case driverless_msgs::msg::State::READY:
            state_str = "READY";
            break;
        case driverless_msgs::msg::State::RELEASE_EBS:
            state_str = "RELEASE_EBS";
            break;
        case driverless_msgs::msg::State::DRIVING:
            state_str = "DRIVING";
            break;
        case driverless_msgs::msg::State::ACTIVATE_EBS:
            state_str = "ACTIVATE_EBS";
            break;
        case driverless_msgs::msg::State::FINISHED:
            state_str = "FINISHED";
            break;
        case driverless_msgs::msg::State::EMERGENCY:
            state_str = "EMERGENCY";
            break;
    }

    return state_str;
}

void ControlGUIPlugin::set_LV_key() {
    state_node_->LV_key_on = ui_.LV_key->value();
    std::cout << "LV_key_on: " << state_node_->LV_key_on << std::endl;
}

void ControlGUIPlugin::set_TS_key() {
    state_node_->TS_key_on = ui_.TS_key->value();
    std::cout << "TS_key_on: " << state_node_->TS_key_on << std::endl;
}

void ControlGUIPlugin::set_AS_key() {
    state_node_->AS_key_on = ui_.AS_key->value();
    std::cout << "AS_key_on: " << state_node_->AS_key_on << std::endl;
}

void ControlGUIPlugin::set_SDC_btn() {
    state_node_->SDC_pressed = true;
    std::cout << "SDC_btn_on: " << state_node_->SDC_pressed << std::endl;
}

void ControlGUIPlugin::set_TS_btn() {
    state_node_->TS_pressed = true;
    std::cout << "TS_btn_on: " << state_node_->TS_pressed << std::endl;
}

void ControlGUIPlugin::set_mission_btn() {
    state_node_->mission_pressed = true;
    std::cout << "mission_btn_on: " << state_node_->mission_pressed << std::endl;
}

void ControlGUIPlugin::set_mission_dropdown() {
    state_node_->selected_mission = ui_.mission_select->currentIndex();
    std::cout << "mission_select: " << state_node_->selected_mission << std::endl;
}

void ControlGUIPlugin::set_estop_btn() {
    if (ui_.estop_btn->isChecked()) {
        state_node_->estop_pressed = true;
    } else {
        state_node_->estop_pressed = false;
    }
    std::cout << "estop_btn_on: " << state_node_->estop_pressed << std::endl;
}

void ControlGUIPlugin::set_r2d_btn() {
    state_node_->r2d_pressed = true;
    std::cout << "r2d_btn_on: " << state_node_->r2d_pressed << std::endl;
}

void ControlGUIPlugin::set_switch_up() {
    state_node_->switch_up = ui_.RES_safety->value();
    std::cout << "RES_safety: " << state_node_->switch_up << std::endl;
}

void ControlGUIPlugin::set_reset_btn() {
    state_node_->reset_pressed = true;
    std::cout << "reset_btn_on: " << state_node_->reset_pressed << std::endl;

    // Reset gui
    ui_.LV_key->setValue(0);
    ui_.TS_key->setValue(0);
    ui_.AS_key->setValue(0);
    ui_.mission_select->setCurrentIndex(0);
    ui_.estop_btn->setChecked(true);
    ui_.RES_safety->setValue(0);
    ui_.laps_box->setValue(0);

    auto request_cones = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_cones = state_node_->reset_cones_srv_->async_send_request(request_cones);
    // Wait for the result
    if (rclcpp::spin_until_future_complete(state_node_, result_cones) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(state_node_->get_logger(), "reset_cones_srv success: %d", result_cones.get()->success);
    } else {
        RCLCPP_ERROR(state_node_->get_logger(), "reset_cones_srv failed");
    }

    auto request_car = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_car = state_node_->reset_car_pos_srv_->async_send_request(request_car);
    // Wait for the result
    if (rclcpp::spin_until_future_complete(state_node_, result_car) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(state_node_->get_logger(), "reset_car_pos_srv success: %d", result_car.get()->success);
    } else {
        RCLCPP_ERROR(state_node_->get_logger(), "reset_car_pos_srv failed");
    }
}

void ControlGUIPlugin::set_laps() {
    std::cout << "laps entered: " << ui_.laps_box->value() << std::endl;
    state_node_->pub_lap_count(ui_.laps_box->value());
}

void ControlGUIPlugin::shutdownPlugin() {
    // Stop ros spin
    ros_timer_->stop();
    rclcpp::shutdown();
}

void ControlGUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                    qt_gui_cpp::Settings& instance_settings) const {
    (void)plugin_settings;
    (void)instance_settings;
}

void ControlGUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                       const qt_gui_cpp::Settings& instance_settings) {
    (void)plugin_settings;
    (void)instance_settings;
}
}  // namespace state_control
PLUGINLIB_EXPORT_CLASS(state_control::ControlGUIPlugin, rqt_gui_cpp::Plugin)
