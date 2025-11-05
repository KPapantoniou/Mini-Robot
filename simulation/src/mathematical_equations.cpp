#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

#define M_PI 3.14159265358979323846

double sign(double x);

// const double max_operating_current = 0.090;
// const double resistant = 26;
// const double omega_rated_speed = (16900*2*M_PI)/60; 
// const double max_start_current = 0.1;
// const double volt = 3.0; 
// const double m = 0.001;
// const double M = 0.05;
// const double m_friction = 0.5;
// const double current = 0.064;
// const double tolerance = 1e-4;
// const double g = 9.81;
// const double r = 0.002;
// const double Kt = (volt-resistant*current)/omega_rated_speed;
// const double c = max_start_current*Kt;
// const double b = (current*Kt-c*sign(omega_rated_speed))/omega_rated_speed;
// const double tmech = 0.033;
// // const double inertia = 0.5*m*r*r; 
// const double inertia = tmech*(pow(Kt,2)+resistant*b)/resistant;

// Paper
const double resistant = 10.7;
const double volt = 0.65; 
const double m = 0.00021;
const double tolerance = 1e-4;
const double g = 9.81;
const double r = 0.002;
const double Kt = 3.64e-4;
const double c = 1.34e-5;
const double b = 2.94e-9;
const double tmech = 0.175;
const double inertia = 2.64e-9;


double step = 0.001;
int iterations = 1500;
double theta = 0;
double omega = 0; 
double fy = 0;
double fz = 0;

double angular_acceleration(double omega, double theta);
double sign(double x);
void rk4_method(double &theta, double &omega);
void horizontal_centrifugal_force(double &fy);
void vertical_centrifugal_force(double &fz);

int main() {
    std::vector<double> theta_values, omega_values, time_values,f_y,f_z;
    std::ofstream file("results.csv");

    file << "Time,Theta,Omega,Force_y,Force_z\n";

    for(int iteration = 0; iteration<iterations; iteration++){
        double t = step*iteration;

        theta_values.push_back(theta);
        omega_values.push_back(omega);
        time_values.push_back(t);
        f_y.push_back(fy);
        f_z.push_back(fz);

        file<< t<<"," <<theta<< "," << omega <<","<<fy<<","<<fz<< "\n";
        
        rk4_method(theta,omega);
        vertical_centrifugal_force(fz);
        horizontal_centrifugal_force(fy);
    }
    
    file.close();
    std::cout<<"Data saved to results.csv"<<std::endl;

    _putenv("PYTHONHOME=");
    int result = system("python ./angular_acceleration_plot.py");

    if (result!=0){
        std::cerr << "Failed to execute angular_acceleration.py" << std::endl;

    }

    return 0;

}

double angular_acceleration(double omega, double theta){
    return (Kt*volt)/(inertia*resistant) -(b*resistant+pow(Kt,2))*omega/(inertia*resistant) - (m*g*r*sin(theta))/ inertia - c*sign(omega)/inertia;
    // return (Kt*curent)/(inertia) -(b*omega)/(inertia) - m*g*r*sin(theta)/ inertia - (m_friction*(M)*g*sign(omega))/inertia;
}

double sign(double x){
    return (x>0)-(x<0);
}

void rk4_method(double &theta, double  &omega){
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

void horizontal_centrifugal_force(double &fy){
    fy = m*pow(omega,2)*r*sin(theta);
}

void vertical_centrifugal_force(double &fz){
    fz = -m*g-m*pow(omega,2)*r*cos(theta);
}