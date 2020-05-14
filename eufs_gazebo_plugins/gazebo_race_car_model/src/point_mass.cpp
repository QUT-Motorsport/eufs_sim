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
namespace fssim {

class PointMass: public VehicleModel {
public:
  PointMass(physics::ModelPtr &_model,
                   sdf::ElementPtr &_sdf,
                   boost::shared_ptr<ros::NodeHandle> &nh,
                   transport::NodePtr &gznode)
    : VehicleModel(_model, _sdf, nh, gznode)
  {
    // IDK
  }

private:
  virtual void updateState(const double dt)
  {
    state_.a_x = input_.dc * std::cos(input_.delta);
    state_.a_y = input_.dc * std::sin(input_.delta);

    State x_dot{};

    x_dot.x = state_.v_x;
    x_dot.y = state_.v_y;

    x_dot.v_x = state_.a_x;
    x_dot.v_y = state_.a_y;

    state_ = state_ + (x_dot * dt);

    state_.yaw = std::atan2(state_.v_y, state_.v_x);
  }
};

} // namespace fssim
} // namespace gazebo
