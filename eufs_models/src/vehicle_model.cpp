#include "eufs_models/vehicle_model.hpp"

namespace eufs {
namespace models {

VehicleModel::VehicleModel(const std::string &yaml_file) : _param(yaml_file) {}

void VehicleModel::validateState(State &state) { state.v_x = std::max(0.0, state.v_x); }

void VehicleModel::validateInput(Input &input) {
  double max_acc = _param.input_ranges.acc.max;
  double min_acc = _param.input_ranges.acc.min;

  double max_vel = _param.input_ranges.vel.max;
  double min_vel = _param.input_ranges.vel.min;

  double max_delta = _param.input_ranges.delta.max;
  double min_delta = _param.input_ranges.delta.min;

  input.acc = std::fmin(std::fmax(input.acc, min_acc), max_acc);
  input.vel = std::fmin(std::fmax(input.vel, min_vel), max_vel);
  input.delta = std::fmin(std::fmax(input.delta, min_delta), max_delta);
}

double VehicleModel::getSlipAngle(const State &x, const Input &u, bool is_front) {
  double lever_arm_length = _param.kinematic.l * _param.kinematic.w_front;

  if (!is_front) {
    double v_x = std::max(1.0, x.v_x);
    return std::atan((x.v_y - lever_arm_length * x.r_z) /
                     (v_x - 0.5 * _param.kinematic.axle_width * x.r_z));
  }

  double v_x = std::max(1.0, x.v_x);
  return std::atan((x.v_y + lever_arm_length * x.r_z) /
                   (v_x - 0.5 * _param.kinematic.axle_width * x.r_z)) -
         u.delta;
}

eufs_msgs::msg::WheelSpeeds VehicleModel::getWheelSpeeds(const State &state, const Input &input) {
  float PI = 3.14159265;
  float wheel_circumference = 2 * PI * _param.tire.radius;

  eufs_msgs::msg::WheelSpeeds wheel_speeds;

  wheel_speeds.steering = input.delta;

  wheel_speeds.lf_speed = 999;
  wheel_speeds.rf_speed = 999;

  // Calculate Wheel speeds
  wheel_speeds.lb_speed = (state.v_x / wheel_circumference) * 60;
  wheel_speeds.rb_speed = (state.v_x / wheel_circumference) * 60;

  return wheel_speeds;
}
}  // namespace models
}  // namespace eufs
