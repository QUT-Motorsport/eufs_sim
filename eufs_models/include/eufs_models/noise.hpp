#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_NOISE_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_NOISE_HPP_

#include <math.h>

#include <string>

#include "eufs_models/vehicle_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "yaml-cpp/yaml.h"

namespace eufs {
namespace models {

struct NoiseParam {
    double position[3];
    double orientation[3];
    double linear_velocity[3];
    double angular_velocity[3];
    double linear_acceleration[3];
    double wheel_speed[4];

    std::string array_to_str(const double arr[], int len) {
        std::string message;
        for (int i = 0; i < len; i++) {
            message += std::to_string(arr[i]);
            message += (i < len - 1) ? ", " : "";
        }

        return "[" + message + "]\n";
    }

    std::string to_str() {
        return "position: " + array_to_str(position, 3) + " orientation: " + array_to_str(orientation, 3) +
               " linear_velocity: " + array_to_str(linear_velocity, 3) +
               " angular_velocity: " + array_to_str(angular_velocity, 3) +
               " linear_acceleration: " + array_to_str(linear_acceleration, 3) +
               " wheel_speed: " + array_to_str(wheel_speed, 4);
    }
};

class Noise {
   public:
    explicit Noise(const std::string &yaml_file) {
        YAML::Node config = YAML::LoadFile(yaml_file);
        _noise_param = config["noise"].as<NoiseParam>();
    }

    State applyNoise(const State &state) {
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

    geometry_msgs::msg::Twist applyNoiseToTwist(const geometry_msgs::msg::Twist &twist) {
        geometry_msgs::msg::Twist new_twist = twist;

        // Add noise to linear velocity
        new_twist.linear.x += _gaussianKernel(0, _noise_param.linear_velocity[0]);
        new_twist.linear.y += _gaussianKernel(0, _noise_param.linear_velocity[1]);
        new_twist.linear.z += _gaussianKernel(0, _noise_param.linear_velocity[2]);

        // Add noise to angular velocity
        new_twist.angular.x += _gaussianKernel(0, _noise_param.angular_velocity[0]);
        new_twist.angular.y += _gaussianKernel(0, _noise_param.angular_velocity[1]);
        new_twist.angular.z += _gaussianKernel(0, _noise_param.angular_velocity[2]);

        return new_twist;
    }

    geometry_msgs::msg::Vector3 applyNoiseToVector(const geometry_msgs::msg::Vector3 &vector) {
        geometry_msgs::msg::Vector3 new_vector = vector;

        // Add noise to linear acceleration
        new_vector.x += _gaussianKernel(0, _noise_param.orientation[0]);
        new_vector.y += _gaussianKernel(0, _noise_param.orientation[1]);
        new_vector.z += _gaussianKernel(0, _noise_param.orientation[2]);

        return new_vector;
    }

    double applyNoiseToSteering(const double &float_msg) {
        double new_float_msg = float_msg;

        // Add noise to linear acceleration
        new_float_msg += _gaussianKernel(0, _noise_param.position[2]);
        // hacky, using yaw to store steering angle noise

        return new_float_msg;
    }

    std::vector<double> applyNoiseToWheels(const std::vector<double> &wheels) {
        std::vector<double> new_wheels = wheels;

        // Add noise to wheel speed
        for (int i = 0; i < 4; i++) {
            new_wheels[i] += _gaussianKernel(0, _noise_param.wheel_speed[i]);
        }

        return new_wheels;
    }

    std::vector<double> noiseToCovariance(const std::vector<double> &covariance) {
        std::vector<double> new_covariance = covariance;

        new_covariance[0] = pow(_noise_param.position[0], 2);
        new_covariance[7] = pow(_noise_param.position[1], 2);
        new_covariance[14] = pow(_noise_param.position[2], 2);

        new_covariance[21] = pow(_noise_param.orientation[0], 2);
        new_covariance[28] = pow(_noise_param.orientation[1], 2);
        new_covariance[35] = pow(_noise_param.orientation[2], 2);

        return new_covariance;
    }

    const NoiseParam &getNoiseParam() { return _noise_param; }

    std::string getString() { return _noise_param.to_str(); }

   private:
    NoiseParam _noise_param;

    // Initialise seed for pseudo-random number generator
    unsigned seed = 0.0;

    double _gaussianKernel(double mu, double sigma) {
        // using Box-Muller transform to generate two independent standard
        // normally distributed normal variables see wikipedia

        // normalized uniform random variable
        double U = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

        // normalized uniform random variable
        double V = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

        double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
        // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

        // there are 2 indep. vars, we'll just use X
        // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
    }
};

}  // namespace models
}  // namespace eufs

namespace YAML {

template <>
struct convert<eufs::models::NoiseParam> {
    static bool decode(const Node &node, eufs::models::NoiseParam &cType) {
        if (node["positionNoise"]) {
            cType.position[0] = node["positionNoise"][0].as<double>();
            cType.position[1] = node["positionNoise"][1].as<double>();
            cType.position[2] = node["positionNoise"][2].as<double>();
        }

        if (node["orientationNoise"]) {
            cType.orientation[0] = node["orientationNoise"][0].as<double>();
            cType.orientation[1] = node["orientationNoise"][1].as<double>();
            cType.orientation[2] = node["orientationNoise"][2].as<double>();
        }

        if (node["linearVelocityNoise"]) {
            cType.linear_velocity[0] = node["linearVelocityNoise"][0].as<double>();
            cType.linear_velocity[1] = node["linearVelocityNoise"][1].as<double>();
            cType.linear_velocity[2] = node["linearVelocityNoise"][2].as<double>();
        }

        if (node["angularVelocityNoise"]) {
            cType.angular_velocity[0] = node["angularVelocityNoise"][0].as<double>();
            cType.angular_velocity[1] = node["angularVelocityNoise"][1].as<double>();
            cType.angular_velocity[2] = node["angularVelocityNoise"][2].as<double>();
        }

        if (node["linearAccelerationNoise"]) {
            cType.linear_acceleration[0] = node["linearAccelerationNoise"][0].as<double>();
            cType.linear_acceleration[1] = node["linearAccelerationNoise"][1].as<double>();
            cType.linear_acceleration[2] = node["linearAccelerationNoise"][2].as<double>();
        }

        if (node["wheelSpeedNoise"]) {
            cType.wheel_speed[0] = node["wheelSpeedNoise"][0].as<double>();
            cType.wheel_speed[1] = node["wheelSpeedNoise"][1].as<double>();
            cType.wheel_speed[2] = node["wheelSpeedNoise"][2].as<double>();
            cType.wheel_speed[3] = node["wheelSpeedNoise"][3].as<double>();
        }

        return true;
    }
};

}  // namespace YAML

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_NOISE_HPP_
