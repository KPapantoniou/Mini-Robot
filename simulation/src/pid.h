#include "motor.h"

#pragma once
// pid.h
#ifndef PID_H
#define PID_H

class PID {
private:
    
    Motor motor;
    double integral, prev_error;

public:
    PID(const Motor& motor);
    double simulate_motor_with_controller(const Parameters& k_parameters, double step, int iterations, double target_omega, bool writeFile, int method);
    double compute_PID(const Parameters& k_parameters, double target, double current, double dt);
    double compute_PD(const Parameters& k_parameters, double target, double current, double dt);
    void reset();
    double compute_volt_pid(const Parameters& k_parameters, double target, double current, double dt);
    double evaluate_cost(const Parameters& k_parameters, double step, int iterations, int target, int method);
    Parameters optimize_parameters(double step, int iterations, int target, int method);
    double simulate_dual_motor_with_controller(
        const Parameters& k_parameters,
        double step,
        int iterations,
        double target_omega,
        bool writeFile,
        Motor& motor_left,
        Motor& motor_right
    );
    double simulate_diff_pid(const Parameters& k_parameters,
        double step,
        int iterations,
        double base_speed,
        double target_diff,
        bool writeFile,
        Motor& motor_left,
        Motor& motor_right
        );
    Parameters optimize_parameters_diff(double step, int iterations, double base_speed, int target, Motor& motor_left, Motor& motor_right);
};

#endif
