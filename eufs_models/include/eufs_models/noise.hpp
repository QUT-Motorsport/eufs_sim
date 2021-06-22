#ifndef EUFS_NOISE_HPP
#define EUFS_NOISE_HPP

#include <math.h>

#include "yaml-cpp/yaml.h"
#include "eufs_models/vehicle_state.hpp"

namespace eufs
{
  namespace models
  {

    struct NoiseParam
    {
      double position[3];
      double orientation[3];
      double linear_velocity[3];
      double angular_velocity[3];
      double linear_acceleration[3];

      std::string array_to_str(const double arr[3])
      {
        return "[" + std::to_string(arr[0]) + "," + std::to_string(arr[1]) + "," + std::to_string(arr[2]) + "]";
      }

      std::string to_str()
      {
        return "position: " + array_to_str(position) +
               " orientation: " + array_to_str(orientation) +
               " linear_velocity: " + array_to_str(linear_velocity) +
               " angular_velocity: " + array_to_str(angular_velocity) +
               " linear_acceleration: " + array_to_str(linear_acceleration);
      }
    };

    class Noise
    {
    public:
      Noise(const std::string &yaml_file)
      {
        YAML::Node config = YAML::LoadFile(yaml_file);
        _noise_param = config["noise"].as<NoiseParam>();
      }

      State applyNoise(const State &state)
      {
        State new_state = state;

        // Add noise to position
        new_state.x += _gaussianKernel(0, _noise_param.position[0]);
        new_state.y += _gaussianKernel(0, _noise_param.position[1]);
        new_state.z += _gaussianKernel(0, _noise_param.position[2]);

        // Add noise to orientation
        new_state.yaw += _gaussianKernel(0, _noise_param.orientation[0]);

        // Add noise to linear velocity
        new_state.v_x += _gaussianKernel(0, _noise_param.linear_velocity[0]);
        new_state.v_y += _gaussianKernel(0, _noise_param.linear_velocity[1]);
        new_state.v_z += _gaussianKernel(0, _noise_param.linear_velocity[2]);

        // Add noise to angular velocity
        new_state.r_x += _gaussianKernel(0, _noise_param.angular_velocity[0]);
        new_state.r_y += _gaussianKernel(0, _noise_param.angular_velocity[1]);
        new_state.r_z += _gaussianKernel(0, _noise_param.angular_velocity[2]);

        // Add noise to linear acceleration
        new_state.a_x += _gaussianKernel(0, _noise_param.linear_acceleration[0]);
        new_state.a_y += _gaussianKernel(0, _noise_param.linear_acceleration[1]);
        new_state.a_z += _gaussianKernel(0, _noise_param.linear_acceleration[2]);

        return new_state;
      }

      const NoiseParam &getNoiseParam() { return _noise_param; }

      std::string getString() { return _noise_param.to_str(); }

    private:
      NoiseParam _noise_param;

      double _gaussianKernel(double mu, double sigma)
      {
        // using Box-Muller transform to generate two independent standard
        // normally distributed normal variables see wikipedia
        unsigned seed = 0;

        // normalized uniform random variable
        double U = static_cast<double>(rand_r(&seed)) /
                   static_cast<double>(RAND_MAX);

        // normalized uniform random variable
        double V = static_cast<double>(rand_r(&seed)) /
                   static_cast<double>(RAND_MAX);

        double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
        // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

        // there are 2 indep. vars, we'll just use X
        // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
      }
    };

  } // namespace models
} // namespace eufs

namespace YAML
{

  template <>
  struct convert<eufs::models::NoiseParam>
  {
    static bool decode(const Node &node, eufs::models::NoiseParam &cType)
    {
      if (node["positionNoise"])
      {
        cType.position[0] = node["positionNoise"][0].as<double>();
        cType.position[1] = node["positionNoise"][1].as<double>();
        cType.position[2] = node["positionNoise"][2].as<double>();
      }

      if (node["orientationNoise"])
      {
        cType.orientation[0] = node["orientationNoise"][0].as<double>();
        cType.orientation[1] = node["orientationNoise"][1].as<double>();
        cType.orientation[2] = node["orientationNoise"][2].as<double>();
      }

      if (node["linearVelocityNoise"])
      {
        cType.linear_velocity[0] = node["linearVelocityNoise"][0].as<double>();
        cType.linear_velocity[1] = node["linearVelocityNoise"][1].as<double>();
        cType.linear_velocity[2] = node["linearVelocityNoise"][2].as<double>();
      }

      if (node["angularVelocityNoise"])
      {
        cType.angular_velocity[0] = node["angularVelocityNoise"][0].as<double>();
        cType.angular_velocity[1] = node["angularVelocityNoise"][1].as<double>();
        cType.angular_velocity[2] = node["angularVelocityNoise"][2].as<double>();
      }

      if (node["linearAccelerationNoise"])
      {
        cType.linear_acceleration[0] = node["linearAccelerationNoise"][0].as<double>();
        cType.linear_acceleration[1] = node["linearAccelerationNoise"][1].as<double>();
        cType.linear_acceleration[2] = node["linearAccelerationNoise"][2].as<double>();
      }

      return true;
    }
  };

} // namespace YAML

#endif //EUFS_NOISE_HPP