#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_DYNAMIC_BICYCLE_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_DYNAMIC_BICYCLE_HPP_

#include <string>
#include "eufs_models/vehicle_model.hpp"

namespace eufs {
namespace models {

class DynamicBicycle : public VehicleModel {
 public:
  explicit DynamicBicycle(const std::string &yaml_file);

  void updateState(State &state, Input &input, const double dt);

 private:
  State _f(const State &x, const Input &u, const double Fx, const double FyF, const double FyR);
  State _fKinCorrection(const State &x_in, const State &x_state, const Input &u, const double Fx,
                        const double dt);
  double _getFx(const State &x, const Input &u);
  double _getNormalForce(const State &x);
  double _getFdown(const State &x);
  double _getFdrag(const State &x);
  double _getFy(const double Fz, bool front, double slip_angle);
  double _getDownForceFront(const double Fz);
  double _getDownForceRear(const double Fz);
};

}  // namespace models
}  // namespace eufs

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_DYNAMIC_BICYCLE_HPP_
