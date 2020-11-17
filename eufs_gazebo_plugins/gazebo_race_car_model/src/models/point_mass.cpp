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

#include "gazebo_race_car_model/vehicle_model.hpp"

namespace gazebo_plugins {
namespace eufs {

class PointMass: public VehicleModel {
public:
  PointMass(gazebo::physics::ModelPtr &_model,
            sdf::ElementPtr &_sdf,
            std::shared_ptr<rclcpp::Node> rosnode,
            gazebo::transport::NodePtr &gznode)
    : VehicleModel(_model, _sdf, rosnode, gznode)
  {}

  virtual void updateState(State& state, Input& input, const double dt)
  {
    state.a_x = input.acc * std::cos(input.delta);
    state.a_y = input.acc * std::sin(input.delta);

    State x_dot{};

    x_dot.x = state.v_x;
    x_dot.y = state.v_y;

    x_dot.v_x = state.a_x;
    x_dot.v_y = state.a_y;

    state = state + (x_dot * dt);

    state.yaw = std::atan2(state.v_y, state.v_x);
  }
};

} // namespace eufs
} // namespace gazebo_plugins
