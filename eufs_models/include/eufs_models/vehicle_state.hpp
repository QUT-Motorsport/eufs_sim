#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_STATE_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_STATE_HPP_

#include <algorithm>
#include <string>

namespace eufs {
namespace models {

struct State {
  State operator*(const double &dt) const {
    return {dt * x,   dt * y,   dt * z,   dt * yaw, dt * v_x, dt * v_y, dt * v_z,
            dt * r_x, dt * r_y, dt * r_z, dt * a_x, dt * a_y, dt * a_z};
  }

  State operator+(const State &x2) const {
    return {x + x2.x,     y + x2.y,     z + x2.z,     yaw + x2.yaw, v_x + x2.v_x,
            v_y + x2.v_y, v_z + x2.v_z, r_x + x2.r_x, r_y + x2.r_y, r_z + x2.r_z,
            a_x + x2.a_x, a_y + x2.a_y, a_z + x2.a_z};
  }

  inline std::string getString() const {
    std::string str = "x:" + std::to_string(x) + "| y:" + std::to_string(y) +
                      "| z:" + std::to_string(z) + "| yaw:" + std::to_string(yaw) +
                      "| v_x:" + std::to_string(v_x) + "| v_y:" + std::to_string(v_y) +
                      "| v_z:" + std::to_string(v_z) + "| r_x:" + std::to_string(r_x) +
                      "| r_y:" + std::to_string(r_y) + "| r_z:" + std::to_string(r_z) +
                      "| a_x:" + std::to_string(a_x) + "| a_y:" + std::to_string(a_y) +
                      "| a_z:" + std::to_string(a_z);
    return str;
  }

  double x = 0;
  double y = 0;
  double z = 0;
  double yaw = 0;
  double v_x = 0;
  double v_y = 0;
  double v_z = 0;
  double r_x = 0;
  double r_y = 0;
  double r_z = 0;
  double a_x = 0;
  double a_y = 0;
  double a_z = 0;
};

}  // namespace models
}  // namespace eufs

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_STATE_HPP_
