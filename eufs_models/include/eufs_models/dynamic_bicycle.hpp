#ifndef EUFS_DYNAMIC_BICYCLE_MODEL_HPP
#define EUFS_DYNAMIC_BICYCLE_MODEL_HPP

#include "eufs_models/vehicle_model.hpp"

namespace eufs
{

  class DynamicBicycle : public VehicleModel
  {
  public:
    DynamicBicycle(Param &param) : VehicleModel(param){};
    DynamicBicycle(std::string &yaml_file) : VehicleModel(yaml_file){};

    void updateState(State &state, Input &input, const double dt) override;

  private:
    double _getSlipAngle(const State &x, const Input &u, bool isFront);
    State _f(const State &x, const Input &u, const double Fx, const double FyF, const double FyR);
    State _f_kin_correction(const State &x_in, const State &x_state, const Input &u, const double Fx, const double dt);
    double _getFx(const State &x, const Input &u);
    double _getNormalForce(const State &x);
    double _getFdown(const State &x);
    double _getFdrag(const State &x);
    double _getFy(const double Fz, bool front, double slipAngle);
    double _getDownForceFront(const double Fz);
    double _getDownForceRear(const double Fz);
  };

}

#endif // EUFS_DYNAMIC_BICYCLE_MODEL_HPP