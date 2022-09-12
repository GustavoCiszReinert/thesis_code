//
// Created by gustavo on 18/06/2022.
//

#include "bicycle_model.h"

bicycle_model::bicycle_model(std::string TIR_file_name) : TIR_file_name(TIR_file_name) {
    std::cout << "Vehicle bicycle model created" << std::endl;

}

void bicycle_model::calculate_slip_angles() {
    beta = v_o / u_o;
    alpha_f = delta_o - (w_o * a + v_o) / u_o;
    alpha_r = (w_o * b - v_o) / u_o;
}

void
bicycle_model::set_vehicle_state(double u_o_in, double v_o_in, double w_o_in, double delta_o_in, double Fx_o_in) {
    u_o = u_o_in;
    v_o = v_o_in;
    w_o = w_o_in;
    delta_o = delta_o_in;
    Fx_o = Fx_o_in;
    calculate_slip_angles();
}

double bicycle_model::f1(const float &R, const float &V_o, const double &u_o_in, const double &v_o_in,
                         const double &w_o_in, const double &delta_o_in, const double &Fx_o_in) {
    return w_o_in - V_o / R;
}

double
bicycle_model::f2(const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
                  const double &delta_o_in, const double &Fx_o_in) {
    return (pow(u_o_in, 2) + pow(v_o_in, 2)) - pow(V_o, 2);
}

double
bicycle_model::f3(const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
                  const double &delta_o_in, const double &Fx_o_in) {
    double alpha_f_in {delta_o_in - (w_o_in * a + v_o_in) / u_o_in};
    tire_front.calculate_output(0, alpha_f_in, (-1 / R), 0, 0, Fz_f, tire_front_output, tire_user_mode, 1);
    return (m * w_o_in * v_o_in) - tire_front_output[1] * sin(delta_o_in) + Fx_o_in;
}

double
bicycle_model::f4(const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
                  const double &delta_o_in, const double &Fx_o_in) {
    double alpha_f_in {delta_o_in - (w_o_in * a + v_o_in) / u_o_in};
    tire_front.calculate_output(0, alpha_f_in, (-1 / R), 0, 0, Fz_f, tire_front_output, tire_user_mode, 1);
    double alpha_r_in {(w_o_in * b - v_o_in) / u_o_in};
    tire_rear.calculate_output(0, alpha_r_in, (-1 / R), 0, 0, Fz_r, tire_rear_output, tire_user_mode, 1);
    return (- m * w_o_in * u_o_in) + tire_front_output[1] * cos(delta_o_in) + tire_rear_output[1];
}

double
bicycle_model::f5(const float &R, const float &V_o, const double &u_o_in, const double &v_o_in, const double &w_o_in,
                  const double &delta_o_in, const double &Fx_o_in) {
    double alpha_f_in {delta_o_in - (w_o_in * a + v_o_in) / u_o_in};
    tire_front.calculate_output(0, alpha_f_in, (-1 / R), 0, 0, Fz_f, tire_front_output, tire_user_mode, 1);
    double alpha_r_in {(w_o_in * b - v_o_in) / u_o_in};
    tire_rear.calculate_output(0, alpha_r_in, (-1 / R), 0, 0, Fz_r, tire_rear_output, tire_user_mode, 1);
    return a * tire_front_output[1] * cos(delta_o_in) - b * tire_rear_output[1];
}
