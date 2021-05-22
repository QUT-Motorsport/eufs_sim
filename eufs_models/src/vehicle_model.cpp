#include "eufs_models/vehicle_model.hpp"

namespace eufs
{

  VehicleModel::VehicleModel(std::string &yaml_file)
  {
    _param = Param(yaml_file);
  }

  void VehicleModel::_validateState(State &state)
  {
    state.v_x = std::max(0.0, state.v_x);
  }

  void VehicleModel::_validateInput(Input &input)
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

  double VehicleModel::_gaussianKernel(double mu, double sigma)
  {
    // using Box-Muller transform to generate two independent standard
    // normally distributed normal variables see wikipedia

    // normalized uniform random variable
    double U = static_cast<double>(rand_r(&seed)) /
               static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&seed)) /
               static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

    // there are 2 indep. vars, we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
  }

} // namespace eufs