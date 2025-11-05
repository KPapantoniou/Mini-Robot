// pid.cpp
#include "pid.h"
#include "iostream"
#include "params.h"
#include <fstream>
#include <random>

using namespace std;

PID::PID(const Motor& motor_)
    :motor(motor_), integral(0.0), prev_error(0.0) {}

double PID::compute_PID(const Parameters& k_parameters, double target, double current, double dt) {
    
    double error = target - current;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return  k_parameters.kp * error + k_parameters.ki * integral + k_parameters.kd*derivative;
}
double PID::compute_PD(const Parameters& k_parameters, double target, double current, double dt) {
    double error = target - current;
    integral += error * dt;
    prev_error = error;

    return -k_parameters.kp * current + k_parameters.ki * integral;
}
void PID::reset() {
    integral = 0.0;
    prev_error = 0.0;
    
}

double PID::compute_volt_pid(const Parameters& k_parameters, double target, double current, double dt) {
    Motor localmotor = motor;
    double out = compute_PID(k_parameters, target, current, dt);
    double control_voltage = ((pow(Kt, 2) + R * b_damp) / Kt) * current + R * cf * localmotor.sign(current) / Kt + J * R * out / Kt + R * m * g * r * sin(localmotor.get_theta());
    control_voltage = std::clamp(control_voltage, -1.5, 1.5);
    return control_voltage;

}

double PID::simulate_motor_with_controller(const Parameters& k_parameters, double step, int iterations, double target_omega, bool writeFile, int method) {

    double error_sum = 0.0;
    Motor localmotor = motor;
    ofstream file1, file2;
    if (writeFile) {
        if (method == 1) {
            file1.open("D:\\panepistimio\\Thesis\\Micro-robot\\code\\\simulation\\sim_pid.csv");
            file1 << "Time,TargetSpeed,Speed,Error,Voltage\n";
        }
        else if (method == 2) {
            file2.open("D:\\panepistimio\\Thesis\\Micro-robot\\code\\\simulation\\sim_pd.csv");
            file2 << "Time,TargetSpeed,Speed,Error,Voltage\n";
        }

    }

    double pid_interval = 0.02;
    int pid_step = static_cast<int>(pid_interval / step);
    double control_voltage = 0.0;
    reset();
    for (int i = 0; i < iterations; ++i) {

        double omega = localmotor.get_omega();
        static float theta = 0.0;
        if (i % pid_step == 0) {
            double out = 0;
            if (method == 1)  out = compute_PID(k_parameters, target_omega, omega, pid_interval);
            else { out = compute_PD(k_parameters, target_omega, omega, pid_interval); }
            theta = omega * (i * step);
            control_voltage = ((pow(Kt, 2) + R * b_damp) / Kt) * omega + R * cf * localmotor.sign(omega) / Kt + J * R * out / Kt + R * m * g * r * sin(theta);
            control_voltage = std::clamp(control_voltage, 0.0, 1.5);
        }

        localmotor.set_voltage(control_voltage);
   
        localmotor.update(step);
 
        double error = target_omega - omega;
        error_sum += error * error;

        if (writeFile && file1.is_open() && (method == 1)) {
            file1 << i * step << "," << target_omega << "," << omega << "," << error << "," << localmotor.get_voltage() << "\n";
        }
        else if (writeFile && file2.is_open() && (method == 2)) {
            file2 << i * step << "," << target_omega << "," << omega << "," << error << "," << localmotor.get_voltage() << "\n";
        }
    }
    if (writeFile && method == 1) file1.close();
    else if (writeFile && method == 2) file2.close();
    return sqrt(error_sum / iterations);
}

double PID::simulate_dual_motor_with_controller(
    const Parameters& k_parameters,
    double step,
    int iterations,
    double target_omega,
    bool writeFile,
    Motor& motor_left,
    Motor& motor_right) {
    

    ofstream file;
    if (writeFile) {       
        file.open("D:\\panepistimio\\Thesis\\Micro-robot\\code\\\simulation\\sim_pid_mean.csv");
        file << "Time,TargetSpeed,Speed_left,Speed_right,Mean_speed,Error,Voltage_left,Voltage_right\n";
        
    }
    double control_voltage_left = 0;
    double control_voltage_right = 0;
    
    double error_sum = 0.0;
    reset();

    double pid_interval = 0.02;
    int pid_step = static_cast<int>(pid_interval / step);

    for (int i = 0; i < iterations; ++i) {
        
        double omega_left = motor_left.get_omega();
        double omega_right = motor_right.get_omega();


        double mean_speed = (omega_left + omega_right) / 2.0;

        static float theta_left = 0.0, theta_right = 0.0;

        if (i % pid_step == 0) {

            double control_mean = compute_PID(k_parameters, target_omega, mean_speed, pid_interval);
            
            double control_left = control_mean;
            double control_right = control_mean;

            theta_left = omega_left * (i * step);
            theta_right = omega_right * (i * step);

            control_voltage_left = ((pow(Kt, 2) + R * b_damp) / Kt) * omega_left
                + R * cf * motor_left.sign(omega_left) / Kt
                + J * R * control_left / Kt
                + R * m * g * r * sin(theta_left);
            control_voltage_right = ((pow(Kt, 2) + R * b_damp) / Kt) * omega_right
                + R * cf * motor_right.sign(omega_right) / Kt
                + J * R * control_right / Kt 
                + R * m * g * r * sin(theta_right);


     
            control_voltage_left = clamp(control_voltage_left, 0.0, 1.5);
            control_voltage_right = clamp(control_voltage_right, 0.0, 1.5);
        }

        motor_left.set_voltage(control_voltage_left);
        motor_right.set_voltage(control_voltage_right);

        motor_left.update(step);
        motor_right.update(step);

        mean_speed = (motor_left.get_omega() + motor_right.get_omega()) / 2.0;
        double error = target_omega - mean_speed;

        error_sum += error * error; 
        if (writeFile && file.is_open()) {
            file << i * step << "," << target_omega << "," << omega_left << ","<<omega_right << "," << mean_speed<<"," << error << "," << motor_left.get_voltage() << "," << motor_right.get_voltage() << endl;
        }

       
    }
    if (writeFile && file.is_open()) file.close();

    return sqrt(error_sum / iterations);  
}

double PID::simulate_diff_pid(const Parameters& k_parameters,
    double step,
    int iterations,
    double base_speed,
    double target_diff,
    bool writeFile,
    Motor& motor_left,
    Motor& motor_right
    ) {

    ofstream file;
   
    if (writeFile) {   
    file.open("D:\\panepistimio\\Thesis\\Micro-robot\\code\\\simulation\\sim_pid_diff.csv");
    file << "Time,TargetSpeed,Speed_left,Speed_right,Diff_speed,Error,Voltage_left,Voltage_right\n";
   
    }
    double control_voltage_left = 0;
    double control_voltage_right = 0;

    double error_sum = 0.0;
    reset();
   

    double pid_interval = 0.02;
    int pid_step = static_cast<int>(pid_interval / step);

    for (int i = 0; i < iterations; ++i) {

        double omega_left = motor_left.get_omega();
        double omega_right = motor_right.get_omega();

        double actual_diff = omega_right - omega_left;
        double error = target_diff - actual_diff;

  

        double left_speed = base_speed - target_diff/2.0;
        double right_speed = base_speed + target_diff/2.0;

        static float theta_left =0, theta_right = 0;

        if (i % pid_step == 0) {
            double control_left = compute_PID(k_parameters, left_speed, omega_left, pid_interval);
            double control_right = compute_PID(k_parameters, right_speed, omega_right, pid_interval);
          
            theta_left = omega_left * (i * step);
            theta_right = omega_right * (i * step);

       
            control_voltage_left = ((pow(Kt, 2) + R * b_damp) / Kt) * omega_left
                + R * cf * motor_left.sign(omega_left) / Kt
                + J * R * (control_left ) / Kt
                + R * m * g * r * sin(theta_left);

            control_voltage_right = ((pow(Kt, 2) + R * b_damp) / Kt) * omega_right
                + R * cf * motor_right.sign(omega_right) / Kt
                + J * R * (control_right)  / Kt 
                + R * m * g * r * sin(theta_right);
            

        

            control_voltage_left = clamp(control_voltage_left, -1.5, 1.5);
            control_voltage_right = clamp(control_voltage_right, -1.5, 1.5);
        }

        motor_left.set_voltage(control_voltage_left);
        motor_right.set_voltage(control_voltage_right);

        motor_left.update(step);
        motor_right.update(step);

      
        double diff_speed_actual = (motor_left.get_omega() - motor_right.get_omega());
 
        double error_ = target_diff - diff_speed_actual;

        error_sum += error * error ;

        if (writeFile && file.is_open()) {
            file << i * step << "," << base_speed << "," << motor_left.get_omega() << "," << motor_right.get_omega() << "," << diff_speed_actual << "," << error<< "," << motor_left.get_voltage() << "," << motor_right.get_voltage() << endl;
        }
    }
    if (writeFile) file.close();
    return sqrt(error_sum / iterations);
    
}
double PID::evaluate_cost(const Parameters& k_parameters, double step, int iterations, int target, int method) {

    double cost = 0.0;
    if (method == 1) cost = simulate_motor_with_controller(k_parameters, step, iterations, target, false, method);
    else if (method == 2) cost = simulate_motor_with_controller(k_parameters, step, iterations, target, false, method);
    return cost;
}

Parameters PID::optimize_parameters(double step, int iterations, int target, int method) {
    motor.reset();
    Parameters best_parametrs = { 0.0,0.0,0.0 };
    Parameters newParameters = best_parametrs;
    double new_cost = 0, best_cost = 0;
    
    best_cost = evaluate_cost(best_parametrs, step, iterations, target, method);
    double temperature = 10.0;
    double cooling_rate = 0.995;
    int max_iterations = 10000;

    std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<double> perturbationKp(-100.0, 100.0);
    std::uniform_real_distribution<double> perturbationKi(-100.0, 100.0);
    std::uniform_real_distribution<double> perturbationKd(-100.0, 100.0);
    std::uniform_real_distribution<double> acceptance(0.0, 1.0);

    std::cout << acceptance(generator) << endl;

    for (int i = 0; i < max_iterations; ++i) {
        
        double step_size = temperature;

        double new_Kp = best_parametrs.kp + perturbationKp(generator) * 0.01;
        double new_Ki = best_parametrs.ki + perturbationKi(generator) * 0.01;
        double new_Kd = best_parametrs.kd + perturbationKd(generator) * 0.01;
        newParameters = { new_Kp, new_Ki, new_Kd };

        newParameters = {
             std::max(0.0, new_Kp),
             method == 2 ? std::max(0.0, new_Ki) : std::min(std::max(0.0, new_Ki), 4.0),
             method == 2 ? std::min(new_Kd, 0.0) : std::min(std::max(0.0, new_Kd), 1.0)
        };
        

        double new_cost = evaluate_cost(newParameters, step, iterations, target, method);


        if (new_cost < best_cost || exp((best_cost - new_cost) / temperature) > acceptance(generator)) {
            best_cost = new_cost;
            best_parametrs.kp = newParameters.kp;
            best_parametrs.ki = newParameters.ki;
            best_parametrs.kd = newParameters.kd;

        }


        temperature *= cooling_rate;
        if (temperature < 0.1) break;
        cout << "Itertations  Controll -> Kp: " << newParameters.kp << " Ki: " << newParameters.ki << " Kd: " << newParameters.kd << " with Cost: " << new_cost << "\n";
        cout << i << "\titeration\t" << temperature << "\ttemperature" << endl;
    }


    std::cout << "Final -> Kp: " << best_parametrs.kp << " Ki: " << best_parametrs.ki << " Kd: " << best_parametrs.kd << " with Cost: " << best_cost << "\n";
    return best_parametrs;
}

Parameters PID::optimize_parameters_diff(double step, int iterations, double base_speed, int target,Motor& motor_left, Motor& motor_right) {
    motor_left.reset();
    motor_right.reset();
    double new_cost = 0, best_cost = 0;
    Parameters best_parametrs = { 0.0,0.0,0.0 };
    Parameters newParameters = best_parametrs;

    best_cost = simulate_diff_pid(best_parametrs,step,iterations,base_speed,target,false,motor_left, motor_right);
    double temperature = 10.0;
    double cooling_rate = 0.995;
    int max_iterations = 10000;

    std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<double> perturbationKp(-100.0, 100.0);
    std::uniform_real_distribution<double> perturbationKi(-100.0, 100.0);
    std::uniform_real_distribution<double> perturbationKd(-100.0, 100.0);
    std::uniform_real_distribution<double> acceptance(0.0, 1.0);

    std::cout << acceptance(generator) << endl;
    int method = 1;
    for (int i = 0; i < max_iterations; ++i) {
        motor_left.reset();
        motor_right.reset();
        double step_size = temperature;

        double new_Kp = best_parametrs.kp + perturbationKp(generator) * 0.001;
        double new_Ki = best_parametrs.ki + perturbationKi(generator) * 0.001;
        double new_Kd = best_parametrs.kd + perturbationKd(generator) * 0.001;
        newParameters = { new_Kp, new_Ki, new_Kd };




        new_cost = simulate_diff_pid(newParameters, step, iterations, base_speed, target, false, motor_left, motor_right);

        if (new_cost < best_cost || exp((best_cost - new_cost) / temperature) > acceptance(generator)) {
            best_cost = new_cost;
            best_parametrs.kp = newParameters.kp;
            best_parametrs.ki = newParameters.ki;
            best_parametrs.kd = newParameters.kd;


        }

        temperature *= cooling_rate;
        if (temperature < 0.1) break;
        cout << "Itertations  Controll -> Kp: " << newParameters.kp << " Ki: " << newParameters.ki << " Kd: " << newParameters.kd << " with Cost: " << new_cost << "\n";
        cout << i << "\titeration\t" << temperature << "\ttemperature" << endl;
    }


    cout << "Final -> Kp: " << best_parametrs.kp << " Ki: " << best_parametrs.ki << " Kd: " << best_parametrs.kd << " with Cost: " << best_cost << "\n";
    return best_parametrs;
}