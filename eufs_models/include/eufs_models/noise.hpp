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
      double position[3] = {0.0, 0.0, 0.0};
      double orientation[3] = {0.0, 0.0, 0.0};
      double linearVelocity[3] = {0.0, 0.0, 0.0};
      double angularVelocity[3] = {0.0, 0.0, 0.0};
      double linearAcceleration[3] = {0.0, 0.0, 0.0};

      std::string array_to_str(const double arr[3])
      {
        return "[" + std::to_string(arr[0]) + "," + std::to_string(arr[1]) + "," + std::to_string(arr[2]) + "]";
      }

      std::string to_str()
      {
        return "position: " + array_to_str(position) +
               " orientation: " + array_to_str(orientation) +
               " linearVelocity: " + array_to_str(linearVelocity) +
               " angularVelocity: " + array_to_str(angularVelocity) +
               " linearAcceleration: " + array_to_str(linearAcceleration);
      }
    };

    class Noise
    {
    public:
      Noise(std::string &yaml_file)
      {
        YAML::Node config = YAML::LoadFile(yaml_file);
        noise_param = config["noise"].as<NoiseParam>();
      }

      State ApplyNoise(const State &state)
      {
        State new_state = state;

        // Add noise to position
        new_state.x += _gaussianKernel(0, noise_param.position[0]);
        new_state.y += _gaussianKernel(0, noise_param.position[1]);
        new_state.z += _gaussianKernel(0, noise_param.position[2]);

        // Add noise to orientation
        new_state.yaw += _gaussianKernel(0, noise_param.orientation[0]);

        // Add noise to linear velocity
        new_state.v_x += _gaussianKernel(0, noise_param.linearVelocity[0]);
        new_state.v_y += _gaussianKernel(0, noise_param.linearVelocity[1]);
        new_state.v_z += _gaussianKernel(0, noise_param.linearVelocity[2]);

        // Add noise to angular velocity
        new_state.r_x += _gaussianKernel(0, noise_param.angularVelocity[0]);
        new_state.r_y += _gaussianKernel(0, noise_param.angularVelocity[1]);
        new_state.r_z += _gaussianKernel(0, noise_param.angularVelocity[2]);

        // Add noise to linear acceleration
        new_state.a_x += _gaussianKernel(0, noise_param.linearAcceleration[0]);
        new_state.a_y += _gaussianKernel(0, noise_param.linearAcceleration[1]);
        new_state.a_z += _gaussianKernel(0, noise_param.linearAcceleration[2]);

        return new_state;
      }

      const NoiseParam &getNoiseParam() { return noise_param; }

      std::string getString() { return noise_param.to_str(); }

    private:
      NoiseParam noise_param;

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
  }
}

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
        cType.linearVelocity[0] = node["linearVelocityNoise"][0].as<double>();
        cType.linearVelocity[1] = node["linearVelocityNoise"][1].as<double>();
        cType.linearVelocity[2] = node["linearVelocityNoise"][2].as<double>();
      }

      if (node["angularVelocityNoise"])
      {
        cType.angularVelocity[0] = node["angularVelocityNoise"][0].as<double>();
        cType.angularVelocity[1] = node["angularVelocityNoise"][1].as<double>();
        cType.angularVelocity[2] = node["angularVelocityNoise"][2].as<double>();
      }

      if (node["linearAccelerationNoise"])
      {
        cType.linearAcceleration[0] = node["linearAccelerationNoise"][0].as<double>();
        cType.linearAcceleration[1] = node["linearAccelerationNoise"][1].as<double>();
        cType.linearAcceleration[2] = node["linearAccelerationNoise"][2].as<double>();
      }

      return true;
    }
  };
}

#endif // EUFS_NOISE_HPP