#include "motor.h"
#include <iostream>

using namespace std;

Motor::Motor(const MotorParams &params)
    : resistance(params.resistance),
      voltage(params.voltage), mass(params.mass),
      g(params.g),
      t_mechanical(params.t_mechanical),
      omega_rated_speed(params.omega_rated_speed),
      start_current(params.start_current),
      operational_current(params.operational_current),
      radius(params.radius),
      omega(0.0),
      theta(0.0){} 

double Motor::angular_acceleration(double omega, double theta) const {    
    return (constant_Torque_calculator() * voltage) / (inertia_calculator() * resistance)
    - ((damping_calculator() * resistance + pow(constant_Torque_calculator(), 2)) * omega) / (inertia_calculator() * resistance)
    - (mass * g * radius * sin(theta)) / inertia_calculator()
    - (coulomb_friction() * sign(omega)) / inertia_calculator();
}

double Motor::sign(double x) const {
    return (x > 0) - (x < 0);
}

double Motor::damping_calculator() const{
    //cout << (operational_current * constant_Torque_calculator() - coulomb_friction() * sign(omega_rated_speed)) / omega_rated_speed << "\tb" << endl;
     //return (operational_current*constant_Torque_calculator()-coulomb_friction()*sign(omega_rated_speed))/omega_rated_speed;
    return 2.61675e-08; //exp
    //return 2.94e-9; //paper
}

double Motor::inertia_calculator() const{
     //return t_mechanical*(pow(constant_Torque_calculator(),2)+resistance*damping_calculator())/resistance;
    //cout << 0.5 * 0.0011 * radius * radius <<"\tJ" << endl;
    return  3.4375e-09; //exp
    //return 2.67e-9; //paper
}

double Motor::coulomb_friction() const{
    //cout << start_current * constant_Torque_calculator()<<"\tc" << endl;
     return 1.17222e-05; //exp
    //return 1.34e-5; //paper
}

double Motor::constant_Torque_calculator() const{
    //cout << (voltage - resistance * operational_current) / omega_rated_speed <<"\tKt" << endl;
     return 0.0008373; //exp
    //return 3.64e-4; //paper
}

void Motor::update(double step){
    // std::cout << "Before update: omega = " << omega << ", theta = " << theta << "\n";

    double k1 = omega*step;
    double l1 = step*(angular_acceleration(omega,theta));

    double k2 = step*(omega +0.5*k1);
    double l2 = step*(angular_acceleration(omega+0.5*l1,theta + 0.5*k1));

    double k3 = step*(omega+0.5*l2);
    double l3 = step*(angular_acceleration(omega+0.5*l2,theta+0.5*k2));

    double k4 = step*(omega+k3);
    double l4 = step*(angular_acceleration(omega+l3,theta + k3));

    theta += (k1+2*k2+2*k3+k4)/6;
    omega += (l1 + 2*l2+ 2*l3+ l4)/6;

}

double Motor::get_omega() const {
    return omega;
}


double Motor::get_theta() const {
    return theta;
}

double Motor::get_voltage() const{
    return voltage;
}

double Motor::get_mass() const{
    return mass;
}

void Motor::set_voltage(double voltage) {
    //cout << voltage << endl;
    this->voltage = voltage;
}

void Motor::reset() {
    omega = 0.0;
    theta = 0.0;
    voltage = 0.0;
}

void Motor::set_omega(double omega) {
    this->omega=omega;
}
void Motor::set_theta(double theta) {
    this->theta = theta;
}