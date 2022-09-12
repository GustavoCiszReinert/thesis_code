#include <iostream>
#include "bicycle_model.h"
// #include "bicycle_model.cpp" // I don't know why I have to add this to build the project without any errors
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void calculate_function_val(float R, float V_o, bicycle_model& veh, Eigen::VectorXd& function_val){
    function_val(0) = veh.f1(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o, veh.Fx_o);
    function_val(1) = veh.f2(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o, veh.Fx_o);
    function_val(2) = veh.f3(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o, veh.Fx_o);
    function_val(3) = veh.f4(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o, veh.Fx_o);
    function_val(4) = veh.f5(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o, veh.Fx_o);
}

void calculate_jacobian_ana(const bicycle_model& veh, Eigen::MatrixXd& jacobian){
    jacobian(0, 0) = 0;
    jacobian(0, 1) = 0;
    jacobian(0, 2) = 1;
    jacobian(0, 3) = 0;
    jacobian(0, 4) = 0;
    jacobian(1, 0) = 2 * veh.u_o;
    jacobian(1, 1) = 2 * veh.v_o;
    jacobian(1, 2) = 0;
    jacobian(1, 3) = 0;
    jacobian(1, 4) = 0;
    jacobian(2, 0) = -veh.C_f / veh.u_o * ((veh.w_o * veh.a + veh.v_o) / veh.u_o) * sin(veh.delta_o);
    jacobian(2, 1) = veh.m * veh.w_o + veh.C_f * sin(veh.delta_o) / veh.u_o;
    jacobian(2, 2) = veh.m * veh.v_o + veh.C_f * veh.a * sin(veh.delta_o) / veh.u_o;
    jacobian(2, 3) = -veh.C_f * sin(veh.delta_o) - veh.C_f * veh.alpha_f * cos(veh.delta_o);
    jacobian(2, 4) = 1;
    jacobian(3, 0) = -veh.m * veh.w_o + veh.C_f / veh.u_o * ((veh.w_o * veh.a + veh.v_o) / veh.u_o)
                                        * cos(veh.delta_o) - veh.C_r / veh.u_o * veh.alpha_r;
    jacobian(3, 1) = -veh.C_f * cos(veh.delta_o) / veh.u_o - veh.C_r / veh.u_o;
    jacobian(3, 2) = -veh.m * veh.u_o - veh.C_f * veh.a * cos(veh.delta_o) / veh.u_o + veh.C_r * veh.b
                                                                                       / veh.u_o;
    jacobian(3, 3) = veh.C_f * cos(veh.delta_o) - veh.C_f * veh.alpha_f * sin(veh.delta_o);
    jacobian(3, 4) = 0;
    jacobian(4, 0) = veh.a * veh.C_f / veh.u_o * ((veh.w_o * veh.a + veh.v_o) / veh.u_o) * cos(veh.delta_o)
                     + veh.b * veh.C_r / veh.u_o * veh.alpha_r;
    jacobian(4, 1) = -veh.a * veh.C_f * cos(veh.delta_o) / veh.u_o + veh.b * veh.C_r / veh.u_o;
    jacobian(4, 2) = -veh.a * veh.C_f * veh.a * cos(veh.delta_o) / veh.u_o - veh.b * veh.C_r * veh.b
                                                                             / veh.u_o;
    jacobian(4, 3) = veh.a * veh.C_f * cos(veh.delta_o) - veh.a * veh.C_f * veh.alpha_f * sin(veh.delta_o);
    jacobian(4, 4) = 0;
}

void calculate_jacobian_num(float R, float V_o, bicycle_model& veh, Eigen::MatrixXd& jacobian, Eigen::VectorXd &function_val_j, double h = 1E-6){
    jacobian(0, 0) = 0;
    jacobian(0, 1) = 0;
    jacobian(0, 2) = (veh.f1(R, V_o, veh.u_o, veh.v_o, veh.w_o + h, veh.delta_o,
                             veh.Fx_o) - function_val_j(0)) / h;
    jacobian(0, 3) = 0;
    jacobian(0, 4) = 0;
    jacobian(1, 0) = (veh.f2(R, V_o, veh.u_o + h, veh.v_o, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(1)) / h;
    jacobian(1, 1) = (veh.f2(R, V_o, veh.u_o, veh.v_o + h, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(1)) / h;
    jacobian(1, 2) = 0;
    jacobian(1, 3) = 0;
    jacobian(1, 4) = 0;
    jacobian(2, 0) = (veh.f3(R, V_o, veh.u_o + h, veh.v_o, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(2)) / h;
    jacobian(2, 1) = (veh.f3(R, V_o, veh.u_o, veh.v_o + h, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(2)) / h;
    jacobian(2, 2) = (veh.f3(R, V_o, veh.u_o, veh.v_o, veh.w_o + h, veh.delta_o,
                             veh.Fx_o) - function_val_j(2)) / h;
    jacobian(2, 3) = (veh.f3(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o + h,
                             veh.Fx_o) - function_val_j(2)) / h;
    jacobian(2, 4) = (veh.f3(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o,
                             veh.Fx_o + h) - function_val_j(2)) / h;
    jacobian(3, 0) = (veh.f4(R, V_o, veh.u_o + h, veh.v_o, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(3)) / h;
    jacobian(3, 1) = (veh.f4(R, V_o, veh.u_o, veh.v_o + h, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(3)) / h;
    jacobian(3, 2) = (veh.f4(R, V_o, veh.u_o, veh.v_o, veh.w_o + h, veh.delta_o,
                             veh.Fx_o) - function_val_j(3)) / h;
    jacobian(3, 3) = (veh.f4(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o + h,
                             veh.Fx_o) - function_val_j(3)) / h;
    jacobian(3, 4) = 0;
    jacobian(4, 0) = (veh.f5(R, V_o, veh.u_o + h, veh.v_o, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(4)) / h;
    jacobian(4, 1) = (veh.f5(R, V_o, veh.u_o, veh.v_o + h, veh.w_o, veh.delta_o,
                             veh.Fx_o) - function_val_j(4)) / h;
    jacobian(4, 2) = (veh.f5(R, V_o, veh.u_o, veh.v_o, veh.w_o + h, veh.delta_o,
                             veh.Fx_o) - function_val_j(4)) / h;
    jacobian(4, 3) = (veh.f5(R, V_o, veh.u_o, veh.v_o, veh.w_o, veh.delta_o + h,
                             veh.Fx_o) - function_val_j(4)) / h;
    jacobian(4, 4) = 0;
}

int main() {
    bicycle_model veh {"../tir_file.txt"};

    // Steady-state Turn
    float R {40}; // Skid pad radius, m
    float V_o {10}; // Constant velocity, m/s

    // Initializing matrix and vector
    VectorXd veh_state (5);
    VectorXd function_val (5);
    MatrixXd jacobian (5,5);
    double step_size {1};
    float tol {1E-3};

    // Initial condition for solution method
    veh.set_vehicle_state(V_o, 0, 0, (veh.w / (10 * R)), 100);
    veh.set_tire_model_mode(2);
    // veh.tire_front.turn_on_TURN_SLIP();
    // veh.tire_rear.turn_on_TURN_SLIP();

    veh_state(0) = veh.u_o;
    veh_state(1) = veh.v_o;
    veh_state(2) = veh.w_o;
    veh_state(3) = veh.delta_o;
    veh_state(4) = veh.Fx_o;

    // Calculate residual error (for the first iteration
    calculate_function_val(R, V_o, veh, function_val);

    do {
        // Calculate Jacobian
        calculate_jacobian_num(R, V_o, veh, jacobian, function_val);
        std::cout << "\nJacobian: \n" << jacobian << std::endl;

        // Next vehicle state
        veh_state = veh_state - step_size * (jacobian.inverse() * function_val);

        // Update vehicle's variables
        veh.set_vehicle_state(veh_state(0), veh_state(1), veh_state(2),
                                 veh_state(3), veh_state(4));

        // Calculate function value / residual error (for the next iteration)
        calculate_function_val(R, V_o, veh, function_val);
    } while (function_val.maxCoeff() > tol);

    std::cout << "\nResults: \n" << veh_state << std::endl;
    std::cout << "\nResidual errors: \n" << function_val << std::endl;
    std::cout << "\nFront tire lateral force: " << veh.C_f*veh.alpha_f << " N" << std::endl;
    std::cout << "\nRear tire lateral force: " << veh.C_r*veh.alpha_r << " N" << std::endl;

    return 0;
}
