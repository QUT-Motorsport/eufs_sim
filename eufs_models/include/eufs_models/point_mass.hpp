#ifndef EUFS_POINT_MASS_MODEL_HPP
#define EUFS_POINT_MASS_MODEL_HPP

#include "eufs_models/vehicle_model.hpp"

namespace eufs
{
  namespace models
  {

    class PointMass : public VehicleModel
    {
    public:
      PointMass(std::string &yaml_file);

      void updateState(State &state, Input &input, const double dt);
    };

  } // namespace models
} // namespace eufs
#endif // EUFS_POINT_MASS_MODEL_HPP