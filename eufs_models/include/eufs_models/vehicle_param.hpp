/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_PARAM_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_PARAM_HPP_

#include <string>
#include "yaml-cpp/yaml.h"

namespace eufs {
namespace models {

struct Param {
  explicit Param(const std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);

    inertia = config["inertia"].as<Param::Inertia>();
    kinematic = config["kinematics"].as<Param::Kinematic>();
    tire = config["tire"].as<Param::Tire>();
    aero = config["aero"].as<Param::Aero>();
    input_ranges = config["input_ranges"].as<Param::InputRanges>();
  }

  struct Inertia {
    double m;
    double g;
    double I_z;
  };

  struct Kinematic {
    double l;
    double b_F;
    double b_R;
    double w_front;
    double l_F;
    double l_R;
    double axle_width;
  };

  struct Tire {
    double tire_coefficient;
    double B;
    double C;
    double D;
    double E;
    double radius;
  };

  struct Aero {
    double c_down;
    double c_drag;
  };

  struct InputRanges {
    struct Range {
      double min;
      double max;
    };

    Range acc;
    Range vel;
    Range delta;
  };

  Inertia inertia;
  Kinematic kinematic;
  Tire tire;
  Aero aero;
  InputRanges input_ranges;
};

}  // namespace models
}  // namespace eufs

namespace YAML {
template <>
struct convert<eufs::models::Param::Inertia> {
  static bool decode(const Node &node, eufs::models::Param::Inertia &cType) {
    cType.m = node["m"].as<double>();
    cType.g = node["g"].as<double>();
    cType.I_z = node["I_z"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Kinematic> {
  static bool decode(const Node &node, eufs::models::Param::Kinematic &cType) {
    cType.l = node["l"].as<double>();
    cType.b_F = node["b_F"].as<double>();
    cType.b_R = node["b_R"].as<double>();
    cType.w_front = node["w_front"].as<double>();
    cType.l_F = cType.l * (1 - cType.w_front);
    cType.l_R = cType.l * cType.w_front;
    cType.axle_width = node["axle_width"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Tire> {
  static bool decode(const Node &node, eufs::models::Param::Tire &cType) {
    cType.tire_coefficient = node["tire_coefficient"].as<double>();
    cType.B = node["B"].as<double>() / cType.tire_coefficient;
    cType.C = node["C"].as<double>();
    cType.D = node["D"].as<double>() * cType.tire_coefficient;
    cType.E = node["E"].as<double>();
    cType.radius = node["radius"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Aero> {
  static bool decode(const Node &node, eufs::models::Param::Aero &cType) {
    cType.c_down = node["C_Down"].as<double>();
    cType.c_drag = node["C_drag"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::InputRanges> {
  static bool decode(const Node &node, eufs::models::Param::InputRanges &cType) {
    cType.acc.min = node["acceleration"]["min"].as<double>();
    cType.acc.max = node["acceleration"]["max"].as<double>();
    cType.vel.min = node["velocity"]["min"].as<double>();
    cType.vel.max = node["velocity"]["max"].as<double>();
    cType.delta.min = node["steering"]["min"].as<double>();
    cType.delta.max = node["steering"]["max"].as<double>();
    return true;
  }
};

}  // namespace YAML

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_PARAM_HPP_
