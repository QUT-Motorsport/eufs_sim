#ifndef EUFS_POINT_MASS_MODEL_HPP
#define EUFS_POINT_MASS_MODEL_HPP

#include "eufs_models/vehicle_model.hpp"

namespace eufs
{

  class PointMass : public VehicleModel
  {
  public:
    PointMass(Param &param) : VehicleModel(param){};
    PointMass(std::string &yaml_file) : VehicleModel(yaml_file){};

    void updateState(State &state, Input &input, const double dt) override;
  };

}

#endif // EUFS_POINT_MASS_MODEL_HPP