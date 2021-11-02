#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_MODEL_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_MODEL_HPP_

#include <memory>
#include <string>

#include "eufs_models/vehicle_input.hpp"
#include "eufs_models/vehicle_param.hpp"
#include "eufs_models/vehicle_state.hpp"
#include "eufs_msgs/msg/wheel_speeds.hpp"

namespace eufs {
namespace models {

class VehicleModel {
 public:
  explicit VehicleModel(const std::string &yaml_file);

  // We need the = 0 at the end otherwise it doesn't seem to work
  virtual void updateState(State &state, Input &input, const double dt) = 0;

  Param &getParam() { return _param; }

  void validateState(State &state);
  void validateInput(Input &input);

  double getSlipAngle(const State &x, const Input &u, bool is_front);
  eufs_msgs::msg::WheelSpeeds getWheelSpeeds(const State &state, const Input &input);

 protected:
  Param _param;
};
typedef std::unique_ptr<VehicleModel> VehicleModelPtr;
}  // namespace models
}  // namespace eufs

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_MODEL_HPP_
