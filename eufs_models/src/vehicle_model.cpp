#include "eufs_models/vehicle_model.hpp"

namespace eufs
{
  namespace models
  {
    VehicleModel::VehicleModel() {}
    VehicleModel::VehicleModel(std::string &yaml_file) : _param(yaml_file) {}

    void VehicleModel::validateState(State &state)
    {
      state.v_x = std::max(0.0, state.v_x);
    }

    void VehicleModel::validateInput(Input &input)
    {
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
  } // namespace models
} // namespace eufs