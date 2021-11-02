#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_POINT_MASS_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_POINT_MASS_HPP_

#include <string>
#include "eufs_models/vehicle_model.hpp"

namespace eufs {
namespace models {

class PointMass : public VehicleModel {
 public:
  explicit PointMass(const std::string &yaml_file);

  void updateState(State &state, Input &input, const double dt);
};

}  // namespace models
}  // namespace eufs

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_POINT_MASS_HPP_
