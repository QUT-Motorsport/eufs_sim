#include "eufs_models/point_mass.hpp"

namespace eufs {
namespace models {

PointMass::PointMass(const std::string &yaml_file) : VehicleModel(yaml_file) {}

void PointMass::updateState(State &state, Input &input, const double dt) {
  validateInput(input);

  state.a_x = input.acc * std::cos(input.delta);
  state.a_y = input.acc * std::sin(input.delta);

  State x_dot{};

  x_dot.x = state.v_x;
  x_dot.y = state.v_y;

  x_dot.v_x = state.a_x;
  x_dot.v_y = state.a_y;

  state = state + (x_dot * dt);

  state.yaw = std::atan2(state.v_y, state.v_x);

  validateState(state);
}

}  // namespace models
}  // namespace eufs
