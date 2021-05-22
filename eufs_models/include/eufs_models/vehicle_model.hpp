#ifndef EUFS_VEHICLE_MODEL_HPP
#define EUFS_VEHICLE_MODEL_HPP

#include "eufs_models/vehicle_input.hpp"
#include "eufs_models/vehicle_param.hpp"
#include "eufs_models/vehicle_state.hpp"

namespace eufs
{

  class VehicleModel
  {
  public:
    VehicleModel(Param &param) : _param(param){};
    VehicleModel(std::string &yaml_file);

    virtual void updateState(State &state, Input &input, const double dt);

    Param &getParam() { return _param; }

  protected:
    Param _param;
    unsigned seed = 0; // For the Gaussian Kernel random number generation

    void _validateInput(Input &input);
    void _validateState(State &state);
    double _gaussianKernel(double mu, double sigma);
  };

  typedef std::unique_ptr<VehicleModel> VehicleModelPtr;

}

#endif // EUFS_VEHICLE_MODEL_HPP