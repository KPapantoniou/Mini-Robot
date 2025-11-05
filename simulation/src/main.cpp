#include "simulation.h"
#include "params.h"
#include "pid.h"
#include "motor.h"

using namespace std;

int main(int argc, char *argv[]) {
    double step = 0.00001;
    int iterations = 100000;

    //exp
    MotorParams motor1Params;

    motor1Params.resistance = 10.71;
    motor1Params.voltage =1;//voltage_right;
    motor1Params.mass = 0.0011;
    motor1Params.g = 9.81;
    motor1Params.t_mechanical = 0.175;
    motor1Params.omega_rated_speed = (11000 * 2 * M_PI) / 60;
    motor1Params.start_current = 0.014;
    motor1Params.operational_current = 0.05;
    motor1Params.radius = 0.0025;



   //paper
    //MotorParams motor1Params;

    //motor1Params.resistance = 10.7;
    //motor1Params.voltage = 1;//voltage_right;
    //motor1Params.mass = 0.00021;
    //motor1Params.g = 9.81;
    //motor1Params.t_mechanical = 0.175;
    //motor1Params.omega_rated_speed = (16000 * 2 * M_PI) / 60;
    //motor1Params.start_current = 0.014;
    //motor1Params.operational_current = 0.05;
    //motor1Params.radius = 0.00177;

    MotorParams motor2Params;
    
    //exp
    motor2Params.resistance = 10.71;
    motor2Params.voltage =0.85;//voltage_left;
    motor2Params.mass = 0.0011;
    motor2Params.g = 9.81;
    //motor2Params.t_mechanical = 0.175;
    motor2Params.omega_rated_speed = (11000 * 2 * M_PI) / 60;
    motor2Params.start_current = 0.014;
    motor2Params.operational_current = 0.05;
    motor2Params.radius = 0.0025;

    //paper
    //motor2Params.resistance = 10.7;
    //motor2Params.voltage = 1;//voltage_left;
    //motor2Params.mass = 0.00021;
    //motor2Params.g = 9.81;
    ////motor2Params.t_mechanical = 0.175;
    //motor2Params.omega_rated_speed = (16000 * 2 * M_PI) / 60;
    //motor2Params.start_current = 0.014;
    //motor2Params.operational_current = 0.05;
    //motor2Params.radius = 0.00177;
    // 
    ForceParams forceParams;

    //exp
     forceParams.mass = 0.0011;
     forceParams.radius=0.0025;
     forceParams.g = 9.81;

    //paper
 /*   forceParams.mass = 0.00021;
    forceParams.radius = 0.00177;
    forceParams.g = 9.81;*/

    RobotParams RobotParams;
    RobotParams.mass = 0.1;
    RobotParams.g = 9.81;
    RobotParams.friction_coefficient = 0.5;
    RobotParams.l = 0.08;

    
    Motor motor_left(motor1Params);
    Motor motor_right(motor2Params);

    PID pid(motor_left);
    int target = 700; 

    /*
    //for SA algorithm to find optimal parameters

    Parameters optimized_pi = pid.optimize_parameters(step, iterations, target, 1);
    Parameters optimized_pid = pid.optimize_parameters(step, iterations, target, 2);
    */
    
    /*
    * for pi - pid controll
    //pi controll experimented parameters
    Parameters PI_parameters = { 43.79, 479.06, 0.0 };
    pid.simulate_motor_with_controller(PI_parameters, step, iterations, 800, true, 2);

    //experimented parameters for pid
    Parameters k_params = {20.9, 2.36, 0.25};
    pid.simulate_motor_with_controller(k_params, step, iterations, 800, true, 1);
    pid.simulate_dual_motor_with_controller(k_params, step, iterations, 900, true, motor_left, motor_right);
    pid.simulate_diff_pid(k_params, step, iterations, 900, 100, true, motor_left, motor_right);

    */
    
    ForceCalculator forceCalculator(forceParams);
    
    Robot robot_single(RobotParams,forceCalculator);
    Robot robot_double(RobotParams,forceCalculator);
    
    robot_single.addMotor(motor_left);
    robot_double.addMotor(motor_left);
    robot_double.addMotor(motor_right);

    Simulation Simulation(step, iterations, motor_left, motor_right, forceCalculator,robot_single,robot_double);
    Simulation.run();

   
    return 0;

}
