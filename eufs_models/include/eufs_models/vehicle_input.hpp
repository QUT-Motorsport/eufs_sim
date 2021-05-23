#ifndef EUFS_VEHICLE_INPUT_HPP
#define EUFS_VEHICLE_INPUT_HPP

#include <string>
#include <math.h>

namespace eufs
{
  namespace models
  {

    struct Input
    {
      Input() : acc(0.0), vel(0.0), delta(0.0) {}

      std::string getString()
      {
        return "acc:" + std::to_string(acc) +
               " | vel:" + std::to_string(vel) +
               " | delta:" + std::to_string(delta);
      }

      double acc;
      double vel;
      double delta;
    };

  } // namespace models
} // namespace eufs
#endif //EUFS_VEHICLE_INPUT_HPP