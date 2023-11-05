#include "vehicle_emulation/input_control_gui.hpp"

#include <memory>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vehicle_emulation {
ControlGUIPlugin::ControlGUIPlugin()
    : rqt_gui_cpp::Plugin(), window_(0), state_node_(std::make_shared<vehicle_emulation::InputProcessingNode>()) {
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

    // convert enum to string
    // update gui
    ui_.VehStateDisplay->setText(QString::fromStdString("veh_state_str"));
    ui_.TSStateDisplay->setText(QString::fromStdString("TS_state_str"));
    ui_.MissionDisplay->setText(QString::fromStdString("supervisor_mission_str"));
    ui_.ASStateDisplay->setText(QString::fromStdString("supervisor_state_str"));
    ui_.LapDisplay->setText(QString::number(0));
}

void ControlGUIPlugin::set_LV_key() {
    state_node_->lv_key_callback(ui_.LV_key->value());
}

void ControlGUIPlugin::set_TS_key() {
    state_node_->ts_key_callback(ui_.TS_key->value());
}

void ControlGUIPlugin::set_AS_key() {
    state_node_->as_key_callback(ui_.AS_key->value());
}

void ControlGUIPlugin::set_SDC_btn() {
    state_node_->sdc_callback(ui_.SDC_btn->isChecked());
}

void ControlGUIPlugin::set_TS_btn() {
    state_node_->ts_callback(ui_.TS_btn->isChecked());
}

void ControlGUIPlugin::set_mission_btn() {
    state_node_->mission_pressed_callback(ui_.mission_btn->isChecked());
}

void ControlGUIPlugin::set_mission_dropdown() {
    state_node_->mission_callback(ui_.mission_select->currentIndex());
}

void ControlGUIPlugin::set_estop_btn() {
    state_node_->estop_callback(ui_.estop_btn->isChecked());
}

void ControlGUIPlugin::set_r2d_btn() {
    state_node_->r2d_callback(ui_.r2d_btn->isChecked());
}

void ControlGUIPlugin::set_switch_up() {
    state_node_->switch_up_callback(ui_.RES_safety->value());
}

void ControlGUIPlugin::set_reset_btn() {
    // Reset gui
    ui_.LV_key->setValue(0);
    ui_.TS_key->setValue(0);
    ui_.AS_key->setValue(0);
    ui_.mission_select->setCurrentIndex(0);
    ui_.estop_btn->setChecked(true);
    ui_.RES_safety->setValue(0);
    ui_.laps_box->setValue(0);

    state_node_->reset_states();

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
}  // namespace vehicle_emulation
PLUGINLIB_EXPORT_CLASS(vehicle_emulation::ControlGUIPlugin, rqt_gui_cpp::Plugin)
