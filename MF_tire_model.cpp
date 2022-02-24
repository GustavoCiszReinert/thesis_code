//
// Created by gusta on 16/02/2022.
//

#include "MF_tire_model.h"

// LONGITUDINAL FORCE, PURE LONGITUDINAL SLIP
double MF_tire_model::Fxo(float Fz, float dfz, float kappa, float gamma, float dpi) {
    double SHx{MF_tire_model::SHx(dfz)};
    double kappa_x{MF_tire_model::kappa_x(kappa, SHx)};
    double Cx{MF_tire_model::Cx()};
    double mux{MF_tire_model::mux(dfz, dpi, gamma)};
    double Dx{MF_tire_model::Dx(mux, Fz)};
    double Ex{MF_tire_model::Ex(dfz, kappa_x)};
    double Kxk{MF_tire_model::Kxk(Fz, dfz, dpi)};
    double Bx{MF_tire_model::Bx(Kxk, Cx, Dx)};
    double SVx{MF_tire_model::SVx(Fz, dfz)};
    return Dx * sin(Cx * atan(Bx * kappa_x - Ex * (Bx * kappa_x - atan(Bx * kappa_x)))) + SVx;}

// LATERAL FORCE, PURE SIDE SLIP
double MF_tire_model::Fyo(float Fz, float dfz, float alpha, float gamma_star, float dpi){
    double Kyg0{MF_tire_model::Kyg0(Fz, dfz, dpi)};
    double Kya{MF_tire_model::Kya(Fz, dpi, gamma_star)};
    double SVyg{MF_tire_model::SVyg(Fz, dfz, gamma_star)};
    double SHy{MF_tire_model::SHy(Kya, Kyg0, SVyg, dfz, gamma_star)};
    double alpha_y{MF_tire_model::alpha_y(alpha, SHy)};
    double Cy{MF_tire_model::Cy()};
    double muy{MF_tire_model::muy(dfz, dpi, gamma_star)};
    double Dy{MF_tire_model::Dy(muy, Fz)};
    double Ey{MF_tire_model::Ey(dfz, gamma_star, alpha_y)};
    double By{MF_tire_model::By(Kya, Cy, Dy)};
    double SVy{MF_tire_model::SVy(SVyg, Fz, dfz)};
    return Dy * sin(Cy * atan(By * alpha_y - Ey * (By * alpha_y - atan(By * alpha_y)))) + SVy;}

// ALIGNING TORQUE PURE SIDE SLIP
double MF_tire_model::Mzo(float Fz, float dfz, float alpha, float gamma_star, float dpi){
    double SHt{MF_tire_model::SHt(dfz, gamma_star)};
    double alpha_t{MF_tire_model::alpha_t(alpha, SHt)};
    double Bt{MF_tire_model::Bt(dfz, gamma_star)};
    double Ct{MF_tire_model::Ct()};
    double Dto{MF_tire_model::Dto(Fz, dfz, dpi)};
    double Dt{MF_tire_model::Dt(Dto, gamma_star)};
    double Et{MF_tire_model::Et(Bt, Ct, alpha_t, dfz, gamma_star)};
    double to{Dt * cos(Ct * atan(Bt * alpha_t - Et * (Bt * alpha_t - atan(Bt * alpha_t))))};

    double muy{MF_tire_model::muy(dfz, dpi, gamma_star)};
    double Dy{MF_tire_model::Dy(muy, Fz)};
    double Kyg0{MF_tire_model::Kyg0(Fz, dfz, dpi)};
    double Kya{MF_tire_model::Kya(Fz, dpi, gamma_star)};
    double SVyg{MF_tire_model::SVyg(Fz, dfz, gamma_star)};
    double SHy{MF_tire_model::SHy(Kya, Kyg0, SVyg, dfz, gamma_star)};
    double SVy{MF_tire_model::SVy(SVyg, Fz, dfz)};
    double Cy{MF_tire_model::Cy()};
    double By{MF_tire_model::By(Kya, Cy, Dy)};
    double SHf{MF_tire_model::SHf(SHy, SVy, Kya)};
    double alpha_r{MF_tire_model::alpha_r(alpha, SHf)};
    double Br{MF_tire_model::Br(By, Cy)};
    double Cr{MF_tire_model::Cr()};
    double Dr{MF_tire_model::Dr(Fz, dfz, dpi, gamma_star)};
    double Mzro{Dr * cos(Cr * atan(Br * alpha_r))};

    double Fyo{MF_tire_model::Fyo(Fz, dfz, alpha, 0, dpi)};
    return (-to * Fyo) + Mzro;} // Fyo for gamma = turn slip = 0

// OVERTURNING COUPLE
double MF_tire_model::Mx(double Fy, float Fz, float gamma, float dpi) const{
    return UNLOADED_RADIUS * Fz * (QSX1 * LVMX - QSX2 * gamma * (1 + PPMX1 * dpi) + QSX3 * (Fy / FNOMIN) + QSX4 *
    cos(QSX5 * atan(pow(QSX6 * (Fz / FNOMIN), 2))) * sin(QSX7 * gamma + QSX8 * atan(QSX9
    * (Fy / FNOMIN))) + QSX10 * atan(QSX11 * (Fz / FNOMIN)) * gamma) * LMX;}

// ROLLING RESISTANCE MOMENT
double MF_tire_model::My(double Fx, float Fz, float gamma, float Vx) const{
    return Fz * UNLOADED_RADIUS * (QSY1 + QSY2 * (Fx / FNOMIN) + QSY3 * abs(Vx / LONGVL) + QSY4 *
    pow((Vx / LONGVL), 4) + (QSY5 + QSY6 * (Fz / FNOMIN)) * pow(gamma, 2)) * (pow((Fz / FNOMIN), QSY7)
    * pow((INFLPRES / NOMPRES), QSY8)) * LMY;}

void MF_tire_model::tire_model_calc(float kappa, float alpha, float Vx, float gamma, float Fz, double MF_output [], int user_mode){

    // MF_OUTPUT [Fx, Fy, My, Mz, Mx]

    float dfz{MF_tire_model::dfz(Fz)};
    float dpi{MF_tire_model::dpi()};
    float gamma_star{MF_tire_model::gamma_star(gamma)};

    if (user_mode == 1) { // longitudinal F&M only: Fx, My
        MF_output[0] = MF_tire_model::Fxo(Fz, dfz, kappa, gamma, dpi);
        MF_output[2] = MF_tire_model::My(MF_output[0], Fz, gamma, Vx);
    }

    if (user_mode == 2) { // lateral F&M only: Fy, Mz, Mx
        MF_output[1] = MF_tire_model::Fyo(Fz, dfz, alpha, gamma_star, dpi);
        MF_output[3] = MF_tire_model::Mzo(Fz, dfz, alpha, gamma_star, dpi);
        MF_output[4] = MF_tire_model::Mx(MF_output[1], Fz, gamma, dpi);
    }
}