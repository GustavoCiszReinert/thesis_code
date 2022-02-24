//
// Created by gusta on 16/02/2022.
//

#ifndef TIRE_MODEL_MF_TIRE_MODEL_H
#define TIRE_MODEL_MF_TIRE_MODEL_H

#include <cmath>

using std::pow;
using std::abs;
using std::copysign;
using std::exp;
using std::sin;
using std::cos;
using std::atan;

class MF_tire_model {
    // Magic Formula tire model v. 6.1.2

    // MODEL
    float LONGVL {}; // Reference velocity

    // DIMENSION
    float UNLOADED_RADIUS {}; // Unloaded tire radius

    // OPERATING_CONDITIONS
    float INFLPRES {}; // Tire inflation pressure, Pa
    const float NOMPRES {}; // Nominal inflation pressure

    // VERTICAL
    const float FNOMIN {}; // Nominal (rated) wheel load

    // SCALING_COEFFICIENTS
    const float LFZO {};
    const float LCX{};
    const float LMUX{};
    const float LEX{};
    const float LKX{};
    const float LHX{};
    const float LVX{};
    const float LCY{};
    const float LMUY{};
    const float LEY{};
    const float LKY{};
    const float LKYC{};
    const float LKZC{};
    const float LHY{};
    const float LVY{};
    const float LTR{};
    const float LRES{};
    const float LXAL{};
    const float LYKA{};
    const float LVYKA{};
    const float LS{};
    const float LMX{};
    const float LVMX{};
    const float LMY{};
    const float LMP{};

    // LONGITUDINAL_COEFFICIENTS
    const double PCX1{};
    const double PDX1{};
    const double PDX2{};
    const double PDX3{};
    const double PEX1{};
    const double PEX2{};
    const double PEX3{};
    const double PEX4{};
    const double PKX1{};
    const double PKX2{};
    const double PKX3{};
    const double PHX1{};
    const double PHX2{};
    const double PVX1{};
    const double PVX2{};
    const double RBX1{};
    const double RBX2{};
    const double RBX3{};
    const double RCX1{};
    const double REX1{};
    const double REX2{};
    const double RHX1{};
    const double PPX1{};
    const double PPX2{};
    const double PPX3{};
    const double PPX4{};

    // OVERTURNING_COEFFICIENTS
    const double QSX1{};
    const double QSX2{};
    const double QSX3{};
    const double QSX4{};
    const double QSX5{};
    const double QSX6{};
    const double QSX7{};
    const double QSX8{};
    const double QSX9{};
    const double QSX10{};
    const double QSX11{};
    const double QSX12{};
    const double QSX13{};
    const double QSX14{};
    const double PPMX1{};

    // LATERAL_COEFFICIENTS
    const double PCY1{};
    const double PDY1{};
    const double PDY2{};
    const double PDY3{};
    const double PEY1{};
    const double PEY2{};
    const double PEY3{};
    const double PEY4{};
    const double PEY5{};
    const double PKY1{};
    const double PKY2{};
    const double PKY3{};
    const double PKY4{};
    const double PKY5{};
    const double PKY6{};
    const double PKY7{};
    const double PHY1{};
    const double PHY2{};
    const double PVY1{};
    const double PVY2{};
    const double PVY3{};
    const double PVY4{};
    const double RBY1{};
    const double RBY2{};
    const double RBY3{};
    const double RBY4{};
    const double RCY1{};
    const double REY1{};
    const double REY2{};
    const double RHY1{};
    const double RHY2{};
    const double RVY1{};
    const double RVY2{};
    const double RVY3{};
    const double RVY4{};
    const double RVY5{};
    const double RVY6{};
    const double PPY1{};
    const double PPY2{};
    const double PPY3{};
    const double PPY4{};
    const double PPY5{};

    // ALIGNING_COEFFICIENTS
    const double QBZ1{};
    const double QBZ2{};
    const double QBZ3{};
    const double QBZ4{};
    const double QBZ5{};
    const double QBZ9{};
    const double QBZ10{};
    const double QCZ1{};
    const double QDZ1{};
    const double QDZ2{};
    const double QDZ3{};
    const double QDZ4{};
    const double QDZ6{};
    const double QDZ7{};
    const double QDZ8{};
    const double QDZ9{};
    const double QDZ10{};
    const double QDZ11{};
    const double QEZ1{};
    const double QEZ2{};
    const double QEZ3{};
    const double QEZ4{};
    const double QEZ5{};
    const double QHZ1{};
    const double QHZ2{};
    const double QHZ3{};
    const double QHZ4{};
    const double SSZ1{};
    const double SSZ2{};
    const double SSZ3{};
    const double SSZ4{};
    const double PPZ1{};
    const double PPZ2{};

    // ROLLING_COEFFICIENTS
    const double QSY1{};
    const double QSY2{};
    const double QSY3{};
    const double QSY4{};
    const double QSY5{};
    const double QSY6{};
    const double QSY7{};
    const double QSY8{};

public:

    //
    float dfz(float Fz) const {return (Fz - LFZO * FNOMIN) / (LFZO * FNOMIN);}
    float dpi() const {return (INFLPRES - NOMPRES) / NOMPRES;}
    float gamma_star(float gamma) {return sin(gamma);}

    // Longitudinal Force (Pure Longitudinal Slip, alpha = 0)
    double kappa_x(float kappa, double SHx) {return kappa + SHx;}
    double Cx() const {return PCX1 * LCX;}
    double Dx(double mux, float Fz, double zeta1 = 1) {return mux * Fz * zeta1;}
    double mux(float dfz, float dpi, float gamma) const {
        return (PDX1 + PDX2 * dfz) * (1 + PPX3 * dpi + PPX4 * pow(dpi, 2)) * (1 - PDX3 * pow(gamma, 2)) * LMUX;}
    double Ex(float dfz, double kappa_x) const {
        return (PEX1 + PEX2 * dfz + PEX3 * pow(dfz, 2)) * (1 - PEX4 * copysign(1.0, kappa_x)) * LEX;}
    double Kxk(float Fz, float dfz, float dpi) const {
        return Fz * (PKX1 + PKX2 * dfz) * exp(PKX3 * dfz) * (1 + PPX1 * dpi + PPX2 * pow(dpi, 2)) * LKX;}
    double Bx(double Kxk, double Cx, double Dx) {return Kxk / (Cx * Dx);}
    double SHx(float dfz) const {return (PHX1 + PHX2 * dfz) * LHX;}
    double SVx(float Fz, float dfz, double zeta1 = 1) const {return Fz * (PVX1 + PVX2 * dfz) * LVX * zeta1;} // degressive friction factor not used
    double Fxo(float Fz, float dfz, float kappa, float gamma, float dpi);

    // Lateral Force (Pure Side Slip, kappa = 0)
    double alpha_y(float alpha, double SHy) {return alpha + SHy;}
    double Cy() const {return PCY1 * LCY;}
    double Dy(double muy, float Fz, double zeta2 = 1) {return muy * Fz * zeta2;}
    double muy(float dfz, float dpi, float gamma_star) const {
        return (PDY1 + PDY2 * dfz) * (1 + PPY3 * dpi + PPY4 * pow(dpi, 2)) * (1 - PDY3 *
        pow(gamma_star, 2)) * LMUY;}
    double Ey (float dfz, float gamma_star, double alpha_y) const {
        return (PEY1 + PEY2 * dfz) * (1 + PEY5 * pow(gamma_star, 2) - (PEY3 + PEY4 * gamma_star) *
        copysign(1.0, alpha_y)) * LEY;}
    double Kya(float Fz, float dpi, float gamma_star, double zeta3 = 1) const {
        return PKY1 * LFZO * FNOMIN * (1 + PPY1 * dpi) * (1 - PKY3 * abs(gamma_star)) * sin(PKY4 *
        atan((Fz / (LFZO * FNOMIN)) / ((PKY2 + PKY5 * pow(gamma_star, 2)) * (1 + PPY2 * dpi)))) * zeta3 * LKY;}
    double By(double Kya, double Cy, double Dy) {return Kya / (Cy * Dy);}
    double SHy(double Kya, double Kyg0, double SVyg, float dfz, float gamma_star, double zeta0 = 1, double zeta4 = 1) const {
        return (PHY1 + PHY2 * dfz) * LHY + (Kyg0 * gamma_star - SVyg) / (Kya) * zeta0 + zeta4 - 1;}
    double SVyg(float Fz, float dfz, float gamma_star, double zeta2 = 1) const {
        return Fz * (PVY3 + PVY4 * dfz) * gamma_star * LKYC * zeta2;} // degressive friction factor not used
    double SVy(double SVyg, float Fz, float dfz, double zeta2 = 1) const {
        return Fz * (PVY1 + PVY2 * dfz) * LVY * zeta2 + SVyg;}
    double Kyg0(float Fz, float dfz, float dpi) const {return Fz * (PKY6 + PKY7 * dfz) * (1 + PPY5 * dpi) * LKYC;}
    double Fyo(float Fz, float dfz, float alpha, float gamma_star, float dpi);

    // Aligning Torque (Pure Side Slip, kappa = 0)
    double alpha_t(float alpha, double SHt) {return alpha + SHt;}
    double SHt(float dfz, float gamma_star) {return QHZ1 + QHZ2 * dfz + (QHZ3 + QHZ4 * dfz) * gamma_star;}
    double alpha_r(float alpha, double SHf) {return alpha + SHf;}
    double SHf(double SHy, double SVy, double Kya) {return SHy + SVy/Kya;}
    double Bt(float dfz, float gamma_star) {
        return (QBZ1 + QBZ2 * dfz + QBZ3 * pow(dfz, 2)) * (1 + QBZ5 * abs(gamma_star) + QBZ6
        * pow(gamma_star, 2)) * LKY / LMUY;}
    double Ct() {return QCZ1;}
    double Dto(float Fz, float dfz, float dpi) {
        return Fz * (UNLOADED_RADIUS / (LFZO * FNOMIN)) * (QDZ1 + QDZ2 * dfz) * (1 - PPZ1 * dpi) * LTR;}
    double Dt(double Dto, float gamma_star, double zeta5 = 1) {
        return Dto * (1 + QDZ3 * std::abs(gamma_star) + QDZ4 * pow(gamma_star, 2)) * zeta5;}
    double Et (double Bt, double Ct, double alpha_t, float dfz, float gamma_star) {
        return (QEZ1 + QEZ2 * dfz + QEZ3 * pow(dfz, 2)) * (1 + (QEZ4 + QEZ5 * gamma_star) *
        (2 / M_PI) * atan(Bt * Ct * alpha_t));}
    double Br(double By, double Cy, double zeta6 = 1) {return (QBZ9 * LKY / LMUY + QBZ10 * By * Cy) * zeta6;}
    double Cr(double zeta7 = 1) {return zeta7;}
    double Dr(float Fz, float dfz, float dpi, float gamma_star, double zeta0 = 1, double zeta2 = 1, double zeta8 = 1) {
        return Fz * UNLOADED_RADIUS * ((QDZ6 + QDZ7 * dfz) * LRES * zeta2 + ((QDZ8 + QDZ9 * dfz) * (1 + PPZ2 * dpi) +
        (QDZ10 + QDZ11 * dfz) * abs(gamma_star)) * gamma_star * LKZC * zeta0) * LMUY + zeta8 -1;}
    double Kzao(double Dto, double Kya) {return Dto * Kya;}
    double Kzgo(double Dto, double Kygo, float Fz, float dfz, float dpi) {
        return Fz * UNLOADED_RADIUS * (QDZ8 + QDZ9 * dfz) * (1 + PPZ2 * dpi) * LKZC * LMUY - Dto * Kygo;}
    double Mzo(float Fz, float dfz, float alpha, float gamma_star, float dpi);
    // Longitudinal Force (Combined Slip)

    // Lateral Force (Combined Slip)

    // Normal Load

    // Overturning Couple
    double Mx(double Fy, float Fz, float gamma, float dpi) const;

    // Rolling Resistance Moment
    double My(double Fx, float Fz, float gamma, float Vx) const;

    // Aligning Torque (Combined Slip)

    void tire_model_calc(float kappa, float alpha, float Vx, float gamma, float Fz, double MF_output [], int user_mode);

};

#endif //TIRE_MODEL_MF_TIRE_MODEL_H
