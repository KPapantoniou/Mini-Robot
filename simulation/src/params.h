#ifndef PARAMS_H
#define PARAMS_H

#include <vector>

#define M_PI 3.14159265358979323846

struct MotorParams {
    double resistance;
    double voltage; 
    double mass;
    double g;
    double t_mechanical;
    double omega_rated_speed;
    double start_current;
    double operational_current;
    double radius;
};

struct ForceParams {
    double mass;
    double radius;
    double g;
};

struct RobotParams{
    double mass;
    double g;
    double friction_coefficient;
    double l;
    double w;
};

struct Parameters {
    double kp;
    double ki;
    double kd;

};

const float Kt = 0.0008373;
const float J = 3.4375e-09;
const float cf = 1.17222e-05;
const float b_damp = 2.61675e-08;
const float R = 10.71;
const float m = 0.0011;
const float g = 9.81;
const float r = 0.0025;
#endif

