//
// Created by gustavo on 18/06/2022.
//

#ifndef TIRE_MODEL_BICYCLE_MODEL_H
#define TIRE_MODEL_BICYCLE_MODEL_H

#include "MF_tire_model.h"

class bicycle_model {
    // Vehicle Bicycle Model

public:

    // Constants
    float g = 9.81; // gravity

    // Dim. and mass
    const float m {720}; // Mass, kg
    const float a {1.8}; // Distance from front axle to CG, m
    const float b {1.6}; // Distance from rear axle to CG, m
    const float w {a + b}; // Wheelbase, m

    // Vehicle state
    double u_o {}; // Velocity's longitudinal component, m/s
    double v_o {}; // Velocity's lateral component, m/s
    double w_o {}; // Yaw rate, rad/s
    double delta_o {}; // Steering angle, rad
    double Fx_o {}; // Rear tire longitudinal force, N

    float Fz_f {(b / w) * m * g}; // Front tire vertical load, N
    float Fz_r {(a / w) * m * g}; // Rear tire vertical load, N
    double beta {}; // chassis slip angle, rad
    double alpha_f {}; // Front slip angle, rad
    double alpha_r {}; // Rear slip angle, rad

    // Tire
    std::string TIR_file_name {};
    MF_tire_model tire_front {TIR_file_name};
    MF_tire_model tire_rear {TIR_file_name};
    double C_f = - tire_front.Kya(Fz_f, 0, 0); // Front cornering stiffness, N/rad
    double C_r = - tire_rear.Kya(Fz_r, 0, 0); // Rear cornering stiffness, N/rad

    // Constructor
    explicit bicycle_model(std::string TIR_file_name);

    // Update vehicle state
    void set_vehicle_state(double u_o_in, double v_o_in, double w_o_in, double delta_o_in, double Fx_o_in);
    void calculate_slip_angles();

    // Cornering, constant speed case, functions
    double f1 (const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
               const double &delta_o_in, const double &Fx_o_in);
    double f2 (const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
               const double &delta_o_in, const double &Fx_o_in);
    double f3 (const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
               const double &delta_o_in, const double &Fx_o_in);
    double f4 (const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
               const double &delta_o_in, const double &Fx_o_in);
    double f5 (const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
               const double &delta_o_in, const double &Fx_o_in);

};


#endif //TIRE_MODEL_BICYCLE_MODEL_H
