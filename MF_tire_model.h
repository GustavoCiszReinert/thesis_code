//
// Created by gustavo on 16/02/2022.
//

#ifndef TIRE_MODEL_MF_TIRE_MODEL_H
#define TIRE_MODEL_MF_TIRE_MODEL_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>

using std::pow;
using std::abs;
using std::copysign;
using std::exp;
using std::sin;
using std::cos;
using std::atan;

class MF_tire_model {
    // Magic Formula tire model v. 6.1.2
    std::string TIR_file_name {};
    std::vector <std::string> TIR_file_content {};

    // MODEL
    double LONGVL {}; // Reference velocity

    // DIMENSION
    double UNLOADED_RADIUS {}; // Unloaded tire radius

    // OPERATING_CONDITIONS
    double INFLPRES {}; // Tire inflation pressure, Pa
    double NOMPRES {}; // Nominal inflation pressure

    // VERTICAL
    double FNOMIN {}; // Nominal (rated) wheel load

    // SCALING_COEFFICIENTS
    double LFZO {};
    double LCX{};
    double LMUX{};
    double LEX{};
    double LKX{};
    double LHX{};
    double LVX{};
    double LCY{};
    double LMUY{};
    double LEY{};
    double LKY{};
    double LKYC{};
    double LKZC{};
    double LHY{};
    double LVY{};
    double LTR{};
    double LRES{};
    double LXAL{};
    double LYKA{};
    double LVYKA{};
    double LS{};
    double LMX{};
    double LVMX{};
    double LMY{};
    double LMP{};

    // TURN SLIP (ZETA) FACTORS
    bool TURN_SLIP_MODE = false; // variable to control whether turn slip is neglected (false) or not (true)
    double ZETA0 {1};
    double ZETA1 {1};
    double ZETA2 {1};
    double ZETA3 {1};
    double ZETA4 {1};
    double ZETA5 {1};
    double ZETA6 {1};
    double ZETA7 {1};
    double ZETA8 {1};

    // LONGITUDINAL_COEFFICIENTS
    double PCX1{};
    double PDX1{};
    double PDX2{};
    double PDX3{};
    double PEX1{};
    double PEX2{};
    double PEX3{};
    double PEX4{};
    double PKX1{};
    double PKX2{};
    double PKX3{};
    double PHX1{};
    double PHX2{};
    double PVX1{};
    double PVX2{};
    double RBX1{};
    double RBX2{};
    double RBX3{};
    double RCX1{};
    double REX1{};
    double REX2{};
    double RHX1{};
    double PPX1{};
    double PPX2{};
    double PPX3{};
    double PPX4{};

    // OVERTURNING_COEFFICIENTS
    double QSX1{};
    double QSX2{};
    double QSX3{};
    double QSX4{};
    double QSX5{};
    double QSX6{};
    double QSX7{};
    double QSX8{};
    double QSX9{};
    double QSX10{};
    double QSX11{};
    double QSX12{};
    double QSX13{};
    double QSX14{};
    double PPMX1{};

    // LATERAL_COEFFICIENTS
    double PCY1{};
    double PDY1{};
    double PDY2{};
    double PDY3{};
    double PEY1{};
    double PEY2{};
    double PEY3{};
    double PEY4{};
    double PEY5{};
    double PKY1{};
    double PKY2{};
    double PKY3{};
    double PKY4{};
    double PKY5{};
    double PKY6{};
    double PKY7{};
    double PHY1{};
    double PHY2{};
    double PVY1{};
    double PVY2{};
    double PVY3{};
    double PVY4{};
    double PPY1{};
    double PPY2{};
    double PPY3{};
    double PPY4{};
    double PPY5{};
    double RBY1{};
    double RBY2{};
    double RBY3{};
    double RBY4{};
    double RCY1{};
    double REY1{};
    double REY2{};
    double RHY1{};
    double RHY2{};
    double RVY1{};
    double RVY2{};
    double RVY3{};
    double RVY4{};
    double RVY5{};
    double RVY6{};

    // ALIGNING_COEFFICIENTS
    double QBZ1{};
    double QBZ2{};
    double QBZ3{};
    double QBZ4{};
    double QBZ5{};
    double QBZ9{};
    double QBZ10{};
    double QCZ1{};
    double QDZ1{};
    double QDZ2{};
    double QDZ3{};
    double QDZ4{};
    double QDZ6{};
    double QDZ7{};
    double QDZ8{};
    double QDZ9{};
    double QDZ10{};
    double QDZ11{};
    double QEZ1{};
    double QEZ2{};
    double QEZ3{};
    double QEZ4{};
    double QEZ5{};
    double QHZ1{};
    double QHZ2{};
    double QHZ3{};
    double QHZ4{};
    double SSZ1{};
    double SSZ2{};
    double SSZ3{};
    double SSZ4{};
    double PPZ1{};
    double PPZ2{};

    // ROLLING_COEFFICIENTS
    double QSY1{};
    double QSY2{};
    double QSY3{};
    double QSY4{};
    double QSY5{};
    double QSY6{};
    double QSY7{};
    double QSY8{};

    // TURNSLIP_COEFFICIENTS

    double PDXP1 = {};
    double PDXP2 = {};
    double PDXP3 = {};
    double PKYP1 = {};
    double PDYP1 = {};
    double PDYP2 = {};
    double PDYP3 = {};
    double PDYP4 = {};
    double PHYP1 = {};
    double PHYP2 = {};
    double PHYP3 = {};
    double PHYP4 = {};
    double PEYP1 = {};
    double PEYP2 = {};
    double QDTP1 = {};
    double QCRP1 = {};
    double QCRP2 = {};
    double QBRP1 = {};
    double QDRP1 = {};

public:

    // CONSTRUCTOR
    explicit MF_tire_model(std::string file_name);

    //
    float dfz(float Fz) const {return (Fz - LFZO * FNOMIN) / (LFZO * FNOMIN);}
    float dpi() const {return (INFLPRES - NOMPRES) / NOMPRES;}
    float gamma_star(float gamma) {return sin(gamma);}

    // Longitudinal Force (Pure Longitudinal Slip, alpha = 0)
    double kappa_x(float kappa, double SHx) {return kappa + SHx;}
    double Cx() const {return PCX1 * LCX;}
    double Dx(double mux, float Fz) {return mux * Fz * ZETA1;}
    double mux(float dfz, float dpi, float gamma) const {
        return (PDX1 + PDX2 * dfz) * (1 + PPX3 * dpi + PPX4 * pow(dpi, 2)) * (1 - PDX3 * pow(gamma, 2)) * LMUX;}
    double Ex(float dfz, double kappa_x) const {
        return (PEX1 + PEX2 * dfz + PEX3 * pow(dfz, 2)) * (1 - PEX4 * copysign(1.0, kappa_x)) * LEX;}
    double Kxk(float Fz, float dfz, float dpi) const {
        return Fz * (PKX1 + PKX2 * dfz) * exp(PKX3 * dfz) * (1 + PPX1 * dpi + PPX2 * pow(dpi, 2)) * LKX;}
    double Bx(double Kxk, double Cx, double Dx) {return Kxk / (Cx * Dx);}
    double SHx(float dfz) const {return (PHX1 + PHX2 * dfz) * LHX;}
    double SVx(float Fz, float dfz) const {return Fz * (PVX1 + PVX2 * dfz) * LVX * ZETA1;} // degressive friction factor not used
    double Fxo(float Fz, float dfz, float kappa, float gamma, float dpi);

    // Lateral Force (Pure Side Slip, kappa = 0)
    double alpha_y(float alpha, double SHy) {return alpha + SHy;}
    double Cy() const {return PCY1 * LCY;}
    double Dy(double muy, float Fz) {return muy * Fz * ZETA2;}
    double muy(float dfz, float dpi, float gamma_star) const {
        return (PDY1 + PDY2 * dfz) * (1 + PPY3 * dpi + PPY4 * pow(dpi, 2)) * (1 - PDY3 *
        pow(gamma_star, 2)) * LMUY;}
    double Ey (float dfz, float gamma_star, double alpha_y) const {
        return (PEY1 + PEY2 * dfz) * (1 + PEY5 * pow(gamma_star, 2) - (PEY3 + PEY4 * gamma_star) *
        copysign(1.0, alpha_y)) * LEY;}
    double Kya(float Fz, float dpi, float gamma_star) const {
        return PKY1 * LFZO * FNOMIN * (1 + PPY1 * dpi) * (1 - PKY3 * abs(gamma_star)) * sin(PKY4 *
        atan((Fz / (LFZO * FNOMIN)) / ((PKY2 + PKY5 * pow(gamma_star, 2)) * (1 + PPY2 * dpi)))) * ZETA3 * LKY;}
    double By(double Kya, double Cy, double Dy) {return Kya / (Cy * Dy);}
    double SHy(double Kya, double Kyg0, double SVyg, float dfz, float gamma_star) const {
        return (PHY1 + PHY2 * dfz) * LHY + (Kyg0 * gamma_star - SVyg) / (Kya) * ZETA0 + ZETA4 - 1;}
    double SVyg(float Fz, float dfz, float gamma_star) const {
        return Fz * (PVY3 + PVY4 * dfz) * gamma_star * LKYC * ZETA2;} // degressive friction factor not used
    double SVy(double SVyg, float Fz, float dfz) const {
        return Fz * (PVY1 + PVY2 * dfz) * LVY * ZETA2 + SVyg;}
    double Kyg0(float Fz, float dfz, float dpi) const {return Fz * (PKY6 + PKY7 * dfz) * (1 + PPY5 * dpi) * LKYC;}
    double Fyo(float Fz, float dfz, float alpha, float phi, float gamma_star, float dpi);

    // Aligning Torque (Pure Side Slip, kappa = 0)
    double alpha_t(float alpha, double SHt) {return alpha + SHt;}
    double SHt(float dfz, float gamma_star) {return QHZ1 + QHZ2 * dfz + (QHZ3 + QHZ4 * dfz) * gamma_star;}
    double alpha_r(float alpha, double SHf) {return alpha + SHf;}
    double SHf(double SHy, double SVy, double Kya) {return SHy + SVy/Kya;}
    double Bt(float dfz, float gamma_star) {
        return (QBZ1 + QBZ2 * dfz + QBZ3 * pow(dfz, 2)) * (1 + QBZ4 * gamma_star + QBZ5 * abs(gamma_star))
        * LKY / LMUY;}
    double Ct() {return QCZ1;}
    double Dto(float Fz, float dfz, float dpi) {
        return Fz * (UNLOADED_RADIUS / (LFZO * FNOMIN)) * (QDZ1 + QDZ2 * dfz) * (1 - PPZ1 * dpi) * LTR;}
    double Dt(double Dto, float gamma_star) {
        return Dto * (1 + QDZ3 * std::abs(gamma_star) + QDZ4 * pow(gamma_star, 2)) * ZETA5;}
    double Et (double Bt, double Ct, double alpha_t, float dfz, float gamma_star) {
        return (QEZ1 + QEZ2 * dfz + QEZ3 * pow(dfz, 2)) * (1 + (QEZ4 + QEZ5 * gamma_star) *
        (2 / M_PI) * atan(Bt * Ct * alpha_t));}
    double Br(double By, double Cy) {return (QBZ9 * LKY / LMUY + QBZ10 * By * Cy) * ZETA6;}
    double Cr() {return ZETA7;}
    double Dr(float Fz, float dfz, float dpi, float gamma_star) {
        return Fz * UNLOADED_RADIUS * ((QDZ6 + QDZ7 * dfz) * LRES * ZETA2 + ((QDZ8 + QDZ9 * dfz) * (1 + PPZ2 * dpi) +
        (QDZ10 + QDZ11 * dfz) * abs(gamma_star)) * gamma_star * LKZC * ZETA0) * LMUY + ZETA8 -1;}
    double Kzao(double Dto, double Kya) {return Dto * Kya;}
    double Kzgo(double Dto, double Kygo, float Fz, float dfz, float dpi) {
        return Fz * UNLOADED_RADIUS * (QDZ8 + QDZ9 * dfz) * (1 + PPZ2 * dpi) * LKZC * LMUY - Dto * Kygo;}
    double Mzo(float Fz, float dfz, float alpha, float phi, float gamma_star, float dpi);
    // Longitudinal Force (Combined Slip)

    // Lateral Force (Combined Slip)

    // Normal Load

    // Overturning Couple
    double Mx(double Fy, float Fz, float gamma, float dpi) const;

    // Rolling Resistance Moment
    double My(double Fx, float Fz, float gamma, float Vx) const;

    // Aligning Torque (Combined Slip)

    // Extension of the model for turn slip
    double Zeta2(float phi, double Byp) {
        return cos(atan(Byp * (UNLOADED_RADIUS * abs(phi) + PDYP4 * sqrt(UNLOADED_RADIUS * abs(phi)))));}
    double Byp(float dfz, float alpha) {return PDYP1 * (1 + PDYP2 * dfz) * cos(atan(PDYP3 * tan(alpha)));}
    double Zeta3(float phi) {return cos(atan(PKYP1 * pow(UNLOADED_RADIUS, 2) * pow(phi, 2)));}
    double SHyp(double CHyp, double DHyp, double EHyp, double BHyp, float phi) {
        return DHyp * sin(CHyp * atan(BHyp * UNLOADED_RADIUS * phi - EHyp * (BHyp * UNLOADED_RADIUS * phi -
                atan(BHyp * UNLOADED_RADIUS * phi))));}
    double Zeta4(double SHyp, double SVyg, double Kya) {return 1 + SHyp - SVyg / Kya;}
    double CHyp() {return PHYP1;}
    double DHyp(float dfz) {return (PHYP2 + PHYP3 * dfz);}
    double EHyp() {return PHYP4;}
    double BHyp(double KyRpo, double CHyp, double DHyp, double Kyao) {return KyRpo / (CHyp * DHyp * Kyao);}
    double KyRpo(double Kyg0, double epsg) {return Kyg0 / (1 - epsg);}
    double epsg(float dfz) {return PEYP1 * (1 + PEYP2 * dfz);}


    void tire_model_calc(float kappa, float alpha, float phi, float Vx, float gamma, float Fz, double MF_output [], int user_mode);

};

#endif //TIRE_MODEL_MF_TIRE_MODEL_H
