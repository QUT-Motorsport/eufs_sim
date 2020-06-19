/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *   - Miguel de la Iglesia Valls <dmiguel@ethz.ch>
 *   - Manuel Dangel <mdangel@student.ethz.ch>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "vehicle_model.hpp"

namespace gazebo {
namespace eufs {

class DynamicBicycle: public VehicleModel {
public:
  DynamicBicycle(physics::ModelPtr &_model,
                   sdf::ElementPtr &_sdf,
                   boost::shared_ptr<ros::NodeHandle> &nh,
                   transport::NodePtr &gznode)
    : VehicleModel(_model, _sdf, nh, gznode)
  {}

private:
  virtual void updateState(State& state, Input& input, const double dt) {
    double Fz = getNormalForce(state);

    double FyF = getFy(Fz, true);
    double FyR = getFy(Fz, false);

    // Drivetrain Model
    const double Fx   = getFx(state, input);
    // Dynamics
    const auto x_dot_dyn  = f(state, input, Fx, FyF, FyR);
    const auto x_next_dyn = state + x_dot_dyn * dt;
    state = f_kin_correction(x_next_dyn, state, input, Fx, dt);

    // Set the acceleration based on the change in velocity
    state.a_x = x_dot_dyn.v_x;
    state.a_y = x_dot_dyn.v_y;

    state.validate();
  }

  State f(const State &x,
                   const Input &u,
                   const double Fx,
                   const double FyF,
                   const double FyR) {
    const double FyF_tot = 2 * FyF;
    const double FyR_tot = 2 * FyR;
    const double v_x     = std::max(1.0, x.v_x);

    State x_dot{};
    x_dot.x   = std::cos(x.yaw) * x.v_x - std::sin(x.yaw) * x.v_y;
    x_dot.y   = std::sin(x.yaw) * x.v_x + std::cos(x.yaw) * x.v_y;
    x_dot.yaw = x.r;
    x_dot.v_x = (x.r * x.v_y) + (Fx - std::sin(u.delta) * FyF_tot) / param_.inertia.m;
    x_dot.v_y = ((std::cos(u.delta) * FyF_tot) + FyR_tot) / param_.inertia.m - (x.r * v_x);
    x_dot.r  = (std::cos(u.delta) * FyF_tot * param_.kinematic.l_F - FyR_tot * param_.kinematic.l_R)
            / param_.inertia.I_z;

    x_dot.a_x = 0;
    x_dot.a_y = 0;

    return x_dot;
  }

  State f_kin_correction(const State &x_in,
                                  const State &x_state,
                                  const Input &u,
                                  const double Fx,
                                  const double dt) {
    State        x       = x_in;
    const double v_x_dot = Fx / (param_.inertia.m);
    const double v       = std::hypot(x_state.v_x, x_state.v_y);
    const double v_blend = 0.5 * (v - 1.5);
    const double blend   = std::fmax(std::fmin(1.0, v_blend), 0.0);

    x.v_x = blend * x.v_x + (1.0 - blend) * (x_state.v_x + dt * v_x_dot);

    const double v_y = std::tan(u.delta) * x.v_x * param_.kinematic.l_R / param_.kinematic.l;
    const double r   = std::tan(u.delta) * x.v_x / param_.kinematic.l;

    x.v_y = blend * x.v_y + (1.0 - blend) * v_y;
    x.r   = blend * x.r + (1.0 - blend) * r;
    return x;
  }

  double getFx(const State &x, const Input &u) {
    const double acc = x.v_x <= 0.0 && u.acc < 0.0 ? 0.0 : u.acc;
    const double Fx = acc * param_.inertia.m - getFdrag(x);
    return Fx;
  }

  double getNormalForce(const State &x) {
    return param_.inertia.g * param_.inertia.m + getFdown(x);
  }

  double getFdown(const State &x) {
    return param_.aero.c_down * x.v_x * x.v_x;
  }

  double getFdrag(const State &x) {
    return param_.aero.c_drag * x.v_x * x.v_x;
  }

  double getFy(const double Fz, bool front) {
    double slipAngle = getSlipAngle(front);

    const double Fz_axle = front ? getDownForceFront(Fz) :
            getDownForceRear(Fz);

    const double B    = param_.tire.B;
    const double C    = param_.tire.C;
    const double D    = param_.tire.D;
    const double E    = param_.tire.E;
    const double mu_y = D * std::sin(C * std::atan(B * (1.0 - E) * slipAngle + E * std::atan(B * slipAngle)));
    const double Fy   = Fz_axle * mu_y;
    return Fy;
    }

  double getDownForceFront(const double Fz) {
      double FzAxle = 0.5 * param_.kinematic.w_front * Fz;
      return FzAxle;
  }

  double getDownForceRear(const double Fz) {
      double FzAxle = 0.5 * (1 - param_.kinematic.w_front) * Fz;
      return FzAxle;
  }
};

} // namespace eufs
} // namespace gazebo
