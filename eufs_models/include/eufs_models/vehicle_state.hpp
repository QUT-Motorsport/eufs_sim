#ifndef EUFS_VEHICLE_STATE_HPP
#define EUFS_VEHICLE_STATE_HPP

#include <string>
#include <algorithm>

namespace eufs
{

  struct State
  {

    State operator*(const double &dt) const
    {
      return {
          dt * x,
          dt * y,
          dt * yaw,
          dt * v_x,
          dt * v_y,
          dt * r,
          dt * a_x,
          dt * a_y};
    }

    State operator+(const State &x2) const
    {
      return {
          x + x2.x,
          y + x2.y,
          yaw + x2.yaw,
          v_x + x2.v_x,
          v_y + x2.v_y,
          r + x2.r,
          a_x + x2.a_x,
          a_y + x2.a_y};
    }

    inline std::string getString() const
    {
      std::string str = "x:" + std::to_string(x) + "| y:" + std::to_string(y) + "| yaw:" + std::to_string(yaw) + "| v_x:" + std::to_string(v_x) + "| v_y:" + std::to_string(v_y) + "| r:" + std::to_string(r) + "| a_x:" + std::to_string(a_x) + "| a_y:" + std::to_string(a_y);
      return str;
    }

    double x;
    double y;
    double yaw;
    double v_x;
    double v_y;
    double r;
    double a_x;
    double a_y;
  };

} // namespace eufs
#endif //EUFS_VEHICLE_STATE_HPP
