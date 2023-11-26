#include "gazebo_steering_motor_plugin/gazebo_ros_steering_motor.hpp"

#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

namespace gazebo_plugins {
namespace eufs_plugins {

SteeringMotorPlugin::SteeringMotorPlugin() {}

SteeringMotorPlugin::~SteeringMotorPlugin() { _update_connection.reset(); }

void SteeringMotorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    _rosnode = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(_rosnode->get_logger(), "Loading SteeringMotorPlugin");

    _model = model;
    _world = _model->GetWorld();

    // Steering joint (only need one)
    std::string leftSteeringJointName = _model->GetName() + "::left_steering_hinge_joint";
    _left_steering_joint = _model->GetJoint(leftSteeringJointName);

    // ROS subscribers
    _sub_can = _rosnode->create_subscription<driverless_msgs::msg::Can>(
        "/can/canbus_carbound", 100, std::bind(&SteeringMotorPlugin::can_rx_callback, this, std::placeholders::_1));

    _pub_can = _rosnode->create_publisher<driverless_msgs::msg::Can>("/can/canopen_rosbound", 100);

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SteeringMotorPlugin::update, this));
    _last_sim_time = _world->SimTime();

    RCLCPP_INFO(_rosnode->get_logger(), "SteeringMotorPlugin Loaded");
}

void SteeringMotorPlugin::can_rx_callback(const driverless_msgs::msg::Can::SharedPtr msg) {
    // receives state control messages from the steering actuator node
    // receives SDO read messages from the steering actuator node

    // check if message is for this node
    if (msg->id != C5E_RX_SRV_ID) {
        return;
    }
    // essentially this is the opposite of the sdo read and write functions

    // check if message is a read (upload) message
    uint16_t object_id = (msg->data[2] & 0xFF) << 8 | (msg->data[1] & 0xFF);
    if (msg->data[0] == 0x40) {
        RCLCPP_DEBUG(_rosnode->get_logger(), "SDO Read: object_id: %x", object_id);
        if (object_id == STATUS_WORD) {
            // create SDO return
            uint32_t id;     // Packet id out
            uint8_t out[8];  // Data out
            uint8_t data[2];
            // arrange status word to be LSB first
            data[0] = _current_state.state_id & 0xFF;
            data[1] = (_current_state.state_id & 0xFF00) >> 8;
            sdo_write(C5E_NODE_ID, object_id, msg->data[3], data, sizeof(data), &id, out);
            _pub_can->publish(_d_2_f(id, 0, out, sizeof(out)));
        }
    } else {
        // last 4 bytes are the position (int32) LSB first
        // iterate to get the 4 bytes
        uint32_t data = 0;
        size_t size = can_open_size_map[msg->data[0]];
        for (size_t i = 0; i < size; i++) {
            data |= (msg->data[4 + i] & 0xFF) << i * 8;
        }
        RCLCPP_DEBUG(_rosnode->get_logger(), "SDO Write: object_id: %x, data: %x", object_id, data);

        // check if message is a write to the control word
        if (object_id == CONTROL_WORD) {
            // compare the control word data to states
            // bits 0-3 are the state control
            uint8_t op_control = data & 0b1111;
            // check if the control word is a state control
            for (auto const& state : states) {
                if (op_control == state.second.control_word) {
                    // check if the state is valid
                    if (state.second.mask & data) {
                        // state is valid
                        _desired_state = state.second;
                    }
                }
            }
            // bit 7 is fault reset
            if (data & 0b10000000) {
                // reset fault
                _desired_state = states[NRTSO];
            }
            // bit 6 is motion type (0 = absolute, 1 = relative)
            if (data & 0b1000000) {
                _motion_profile_type = 1;
            } else {
                _motion_profile_type = 0;
            }
            // bit 4 is trigger motion
            if (data & 0b10000) {
                _moving = true;
            } else {
                _moving = false;
            }
        } else if (object_id == TARGET_POSITION) {
            // target position
            _target_position = data;
        } else if (object_id == PROFILE_VELOCITY) {
            // profile velocity
            _profile_velocity = data;
        } else if (object_id == END_VELOCITY) {
            // end velocity
            _end_velocity = data;
        } else if (object_id == PROFILE_ACCELERATION) {
            // profile acceleration
            _profile_acceleration = data;
        } else if (object_id == PROFILE_DECELERATION) {
            // profile deceleration
            _profile_deceleration = data;
        } else if (object_id == QUICK_STOP_DECELERATION) {
            // quick stop deceleration
            _quick_stop_deceleration = data;
        } else if (object_id == MAX_ACCELERATION) {
            // max acceleration
            _max_acceleration = data;
        } else if (object_id == MAX_DECELERATION) {
            // max deceleration
            _max_deceleration = data;
        } else if (object_id == MODE_OF_OPERATION) {
            // mode of operation
            _mode_of_operation = data;
        } else if (object_id == HOME_OFFSET) {
            // home offset
            _home_offset = data;
        }
        uint8_t return_data[4] = {0, 0, 0, 0};
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out
        sdo_write(C5E_NODE_ID, object_id, 0x60, return_data, sizeof(data), &id, out);
        _pub_can->publish(_d_2_f(id, 0, out, sizeof(out)));
    }
}

void SteeringMotorPlugin::update() {
    // Check against update rate
    gazebo::common::Time curTime = _world->SimTime();
    double dt = calc_dt(_last_sim_time, curTime);
    if (dt < (1 / _update_rate)) {
        return;
    }

    // log state attributes every 1 second
    RCLCPP_INFO_THROTTLE(_rosnode->get_logger(), *_rosnode->get_clock(), 5000,
                         "Steering Motor State: \n \
        current_state: %s \n \
        desired_state: %s \n \
        home_offset: %s \n \
        motion_profile_type: %s \n \
        profile_velocity: %s \n \
        end_velocity: %s \n \
        profile_acceleration: %s \n \
        profile_deceleration: %s \n \
        quick_stop_deceleration: %s \n \
        max_acceleration: %s \n \
        max_deceleration: %s",
                         _current_state.name.c_str(), _desired_state.name.c_str(), std::to_string(_home_offset).c_str(),
                         std::to_string(_motion_profile_type).c_str(), std::to_string(_profile_velocity).c_str(),
                         std::to_string(_end_velocity).c_str(), std::to_string(_profile_acceleration).c_str(),
                         std::to_string(_profile_deceleration).c_str(),
                         std::to_string(_quick_stop_deceleration).c_str(), std::to_string(_max_acceleration).c_str(),
                         std::to_string(_max_deceleration).c_str());

    // get steering angle
    float steering_angle = _left_steering_joint->Position(0);
    // convert rads to linear encoder steps (-0.28, 0.28) = (-7500, 7500)
    _position_actual_value = (int32_t)(steering_angle * 7500 / 0.28);

    // publish boot msg once
    if (_current_state == states[NRTSO]) {
        RCLCPP_INFO_ONCE(_rosnode->get_logger(), "Steering Motor Booted");
        uint8_t out[1];  // Data out
        // fill with 0s
        out[0] = 0;
        _pub_can->publish(_d_2_f(C5E_TX_BOOT_UP_ID, 0, out, sizeof(out)));
    }

    // update state with state machine, transition up to desired state
    // use state.step to transition to next state. only up one state at a time
    if (_current_state.step == (_desired_state.step - 1)) {
        update_state();
    } else if (_desired_state.step < _current_state.step) {
        // transition down as many states as needed
        update_state();
    }

    if (_current_state.step >= states[SOD].step) {
        // can publish encoder position if different to last position
        if (_position_actual_value != _last_position_actual_value) {
            _pub_can->publish(_d_2_f(C5E_TX_POS_ID, 0, (uint8_t*)&_position_actual_value, 4));
        }
    }

    if (_current_state == states[OE]) {
        // commands to move
        if (_motion_profile_type == 0) {
            RCLCPP_INFO_ONCE(_rosnode->get_logger(), "Moving relative");
        } else if (_motion_profile_type == 1) {
            RCLCPP_INFO_ONCE(_rosnode->get_logger(), "Moving absolute");
        }

        if (_moving) {
            RCLCPP_DEBUG(_rosnode->get_logger(), "Triggering new target position");
            // move to target position
            _moving = false;
        }
    }

    // fault
    // if (_current_state == states[F]) {
    //     // will need to wait for a fault reset command
    //     if (_desired_state == states[NRTSO]) {
    //         _current_state = states[NRTSO];
    //     }
    // }

    // publish status word and mode of op together
    // status word is bit 1 and 0, mode of op is bit 2
    //             uint16_t status_word = (msg.data[1] << 8 | msg.data[0]);
    //             uint16_t mode_of_operation = msg.data[2];
    // pack data this way

    // update last values
    _last_sim_time = curTime;
    _last_position_actual_value = _position_actual_value;
}

void SteeringMotorPlugin::update_state() {
    // update state with state machine, transition up to desired state
    _current_state = _desired_state;

    uint32_t data = 0;
    // arrange status word to be LSB first
    data |= _current_state.state_id & 0xFF;
    data |= (_current_state.state_id & 0xFF00) >> 8;
    // mode of operation
    data |= _mode_of_operation << 16;
    _pub_can->publish(_d_2_f(C5E_TX_STATUS_ID, 0, (uint8_t*)&data, 4));
}

GZ_REGISTER_MODEL_PLUGIN(SteeringMotorPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
