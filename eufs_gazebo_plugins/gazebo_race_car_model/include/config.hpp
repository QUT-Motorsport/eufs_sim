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

#ifndef GAZEBO_CONFIG_HPP
#define GAZEBO_CONFIG_HPP
#include "ros/common.h"
#include "yaml-cpp/yaml.h"

struct Param {
    struct Inertia {
        double m;
        double g;
        double I_z;
        void print() {
            ROS_DEBUG("Inertia: \n "
                      "\tm: %f\n"
                      "\tg: %f\n"
                      "\tI_z: %f", m, g, I_z);
        }
    };

    struct Kinematic {
        double l;
        double b_F;
        double b_R;
        double w_front;
        double l_F;
        double l_R;
        void print() {
            ROS_DEBUG("Kinematic: \n "
                      "\tl: %f\n"
                      "\tb_F: %f\n"
                      "\tb_R: %f\n"
                      "\tw_front: %f\n"
                      "\tl_F: %f\n"
                      "\tl_R: %f\n", l, b_F, b_R, w_front, l_F, l_R);
        }
    };

    struct Tire {
        double tire_coefficient;
        double B;
        double C;
        double D;
        double E;
        double radius;
        void print() {
            ROS_DEBUG("Tire: \n "
                      "\tB: %f\n"
                      "\tC: %f\n"
                      "\tD: %f\n"
                      "\tE: %f\n"
                      "\tradius: %f", B, C, D, E, radius);
        }
    };

    struct Aero {
        double c_down;
        double c_drag;
        void print() {
            ROS_DEBUG("Aero: \n "
                      "\tc_down: %f\n"
                      "\tc_drag: %f", c_down, c_drag);
        }
    };

    struct InputRanges {
        struct Range {
            double min;
            double max;
            void print() {
                ROS_DEBUG("\tmin: %f"
                          "\tmax: %f", min, max);
            }
        };

        Range acc;
        Range delta;
        void print() {
            ROS_DEBUG("Input ranges: ");
            ROS_DEBUG("acc: ");
            acc.print();
            ROS_DEBUG("delta: ");
            delta.print();
        }
    };

    Inertia         inertia;
    Kinematic       kinematic;
    Tire            tire;
    Aero            aero;
    InputRanges     input_ranges;
};

namespace YAML {
template<>
struct convert<Param::Inertia> {
    static bool decode(const Node &node, Param::Inertia &cType) {
        cType.m        = node["m"].as<double>();
        cType.g        = node["g"].as<double>();
        cType.I_z      = node["I_z"].as<double>();
        ROS_DEBUG("LOADED Inertia");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Kinematic> {
    static bool decode(const Node &node, Param::Kinematic &cType) {
        cType.l       = node["l"].as<double>();
        cType.b_F     = node["b_F"].as<double>();
        cType.b_R     = node["b_R"].as<double>();
        cType.w_front = node["w_front"].as<double>();
        cType.l_F     = cType.l * (1 - cType.w_front);
        cType.l_R     = cType.l * cType.w_front;
        ROS_DEBUG("LOADED Kinematic");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Tire> {
    static bool decode(const Node &node, Param::Tire &cType) {
        cType.tire_coefficient = node["tire_coefficient"].as<double>();
        cType.B = node["B"].as<double>() / cType.tire_coefficient;
        cType.C = node["C"].as<double>();
        cType.D = node["D"].as<double>() * cType.tire_coefficient;
        cType.E = node["E"].as<double>();
        cType.radius = node["radius"].as<double>();
        ROS_DEBUG("LOADED Tire");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Aero> {
    static bool decode(const Node &node, Param::Aero &cType) {
        cType.c_down = node["C_Down"].as<double>();
        cType.c_drag = node["C_drag"].as<double>();
        ROS_DEBUG("LOADED Aero");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::InputRanges> {
    static bool decode(const Node &node, Param::InputRanges &cType) {
        cType.acc.min   = node["acceleration"]["min"].as<double>();
        cType.acc.max   = node["acceleration"]["max"].as<double>();
        cType.delta.min = node["steering"]["min"].as<double>();
        cType.delta.max = node["steering"]["max"].as<double>();
        ROS_DEBUG("LOADED InputRanges");
        cType.print();
        return true;
    }
};

}

inline void initParamStruct(Param &param, std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);
    ROS_DEBUG("STARTING THIS YAML CraP: %s ********************************", yaml_file.c_str());

    param.inertia         = config["inertia"].as<Param::Inertia>();
    param.kinematic       = config["kinematics"].as<Param::Kinematic>();
    param.tire            = config["tire"].as<Param::Tire>();
    param.aero            = config["aero"].as<Param::Aero>();
    param.input_ranges    = config["input_ranges"].as<Param::InputRanges>();
}

#endif //GAZEBO_CONFIG_HPP
