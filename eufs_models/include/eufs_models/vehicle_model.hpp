#ifndef EUFS_VEHICLE_MODEL_HPP
#define EUFS_VEHICLE_MODEL_HPP

#include <memory>
#include "eufs_models/vehicle_input.hpp"
#include "eufs_models/vehicle_param.hpp"
#include "eufs_models/vehicle_state.hpp"

namespace eufs
{
  namespace models
  {

    class VehicleModel
    {
    public:
      VehicleModel();
      VehicleModel(std::string &yaml_file);

      virtual void updateState(State &state, Input &input, const double dt) = 0;

      Param &getParam() { return _param; }

      void validateInput(Input &input);
      void validateState(State &state);

    protected:
      Param _param;
    };

    typedef std::unique_ptr<VehicleModel> VehicleModelPtr;

  } // namespace models
} // namespace eufs

#endif // EUFS_VEHICLE_MODEL_HPP