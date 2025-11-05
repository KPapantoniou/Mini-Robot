#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include "motor.h"
#include "force_calculator.h"
#include "robot.h"
#include <vector>
#include <fstream>
#include "pid.h"
#include <random>
#include <cmath>

class Simulation{
public:
    Simulation(double step, int iterations, Motor& motor1, Motor& motor2, ForceCalculator& forceCalc, Robot& robot_single, Robot& robot_double);
    void run_planned_trajectory(const std::vector<std::pair<double, double>>& trajectory_points, PID& pid_controller);
    void run();
private:
    double step;
    int iterations;
    Motor* motor1;
    Motor* motor2;
    ForceCalculator forceCalc;
    Robot* robot_single;
    Robot* robot_double;
    // double theta, omega;
};
#endif