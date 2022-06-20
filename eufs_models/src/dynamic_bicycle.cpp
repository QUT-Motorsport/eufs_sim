#include "eufs_models/dynamic_bicycle.hpp"

namespace eufs {
namespace models {

DynamicBicycle::DynamicBicycle(const std::string &yaml_file) : VehicleModel(yaml_file) {}

void DynamicBicycle::updateState(State &state, Input &input, const double dt) {
  validateInput(input);

  double Fz = _getNormalForce(state);

  double slip_angle_front = getSlipAngle(state, input, true);
  double FyF = _getFy(Fz, true, slip_angle_front);

  double slip_angle_back = getSlipAngle(state, input, false);
  double FyR = _getFy(Fz, false, slip_angle_back);

  // Drivetrain Model
  const double Fx = _getFx(state, input);
  // Dynamics
  const auto x_dot_dyn = _f(state, input, Fx, FyF, FyR);
  const auto x_next_dyn = state + x_dot_dyn * dt;
  state = _fKinCorrection(x_next_dyn, state, input, Fx, dt);

  // Set the acceleration based on the change in velocity
  state.a_x = x_dot_dyn.v_x;
  state.a_y = x_dot_dyn.v_y;

  validateState(state);
}

State DynamicBicycle::_f(const State &x, const Input &u, const double Fx, const double FyF,
                         const double FyR) {
  const double FyF_tot = 2 * FyF;
  const double FyR_tot = 2 * FyR;

  State x_dot{};

  x_dot.x = std::cos(x.yaw) * x.v_x - std::sin(x.yaw) * x.v_y;
  x_dot.y = std::sin(x.yaw) * x.v_x + std::cos(x.yaw) * x.v_y;

  x_dot.yaw = x.r_z;

  x_dot.v_x = (x.r_z * x.v_y) + (Fx - std::sin(u.delta) * FyF_tot) / _param.inertia.m;
  x_dot.v_y = ((std::cos(u.delta) * FyF_tot) + FyR_tot) / _param.inertia.m - (x.r_z * x.v_x);

  x_dot.r_z =
      (std::cos(u.delta) * FyF_tot * _param.kinematic.l_F - FyR_tot * _param.kinematic.l_R) /
      _param.inertia.I_z;

  return x_dot;
}

State DynamicBicycle::_fKinCorrection(const State &x_in, const State &x_state, const Input &u,
                                      const double Fx, const double dt) {
  State x = x_in;
  const double v_x_dot = Fx / (_param.inertia.m);
  const double v = std::hypot(x_state.v_x, x_state.v_y);
  const double v_blend = 0.5 * (v - 1.5);
  const double blend = std::fmax(std::fmin(1.0, v_blend), 0.0);

  x.v_x = blend * x.v_x + (1.0 - blend) * (x_state.v_x + dt * v_x_dot);

  const double v_y = std::tan(u.delta) * x.v_x * _param.kinematic.l_R / _param.kinematic.l;
  const double r = std::tan(u.delta) * x.v_x / _param.kinematic.l;

  x.v_y = blend * x.v_y + (1.0 - blend) * v_y;
  x.r_z = blend * x.r_z + (1.0 - blend) * r;
  return x;
}

double DynamicBicycle::_getFx(const State &x, const Input &u) {
  const double acc = x.v_x <= 0.0 && u.acc < 0.0 ? 0.0 : u.acc;
  const double Fx = acc * _param.inertia.m - _getFdrag(x);
  return Fx;
}

double DynamicBicycle::_getNormalForce(const State &x) {
  return _param.inertia.g * _param.inertia.m + _getFdown(x);
}

double DynamicBicycle::_getFdown(const State &x) { return _param.aero.c_down * x.v_x * x.v_x; }

double DynamicBicycle::_getFdrag(const State &x) { return _param.aero.c_drag * x.v_x * x.v_x; }

double DynamicBicycle::_getFy(const double Fz, bool front, double slip_angle) {
  const double Fz_axle = front ? _getDownForceFront(Fz) : _getDownForceRear(Fz);

  const double B = _param.tire.B;
  const double C = _param.tire.C;
  const double D = _param.tire.D;
  const double E = _param.tire.E;
  const double mu_y =
      D * std::sin(C * std::atan(B * (1.0 - E) * slip_angle + E * std::atan(B * slip_angle)));
  const double Fy = Fz_axle * mu_y;
  return Fy;
}

double DynamicBicycle::_getDownForceFront(const double Fz) {
  double FzAxle = 0.5 * _param.kinematic.w_front * Fz;
  return FzAxle;
}

double DynamicBicycle::_getDownForceRear(const double Fz) {
  double FzAxle = 0.5 * (1 - _param.kinematic.w_front) * Fz;
  return FzAxle;
}

}  // namespace models
}  // namespace eufs
