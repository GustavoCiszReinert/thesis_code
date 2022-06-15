//
// Created by gusta on 16/02/2022.
//

#include "MF_tire_model.h"
#include "import_TIR.h"

// CONSTRUCTOR
MF_tire_model::MF_tire_model(std::string file_name) : TIR_file_name(file_name) {
    std::cout << "Initializing tire model" << std::endl;
    import_TIR_file(TIR_file_name, TIR_file_content);

    // MODEL
    LONGVL = get_coefficient(TIR_file_content, "LONGVL"); // Reference velocity

    // DIMENSION
    UNLOADED_RADIUS = get_coefficient(TIR_file_content, "UNLOADED_RADIUS"); // Unloaded tire radius

    // OPERATING_CONDITIONS
    INFLPRES = get_coefficient(TIR_file_content, "INFLPRES"); // Tire inflation pressure, Pa
    NOMPRES = get_coefficient(TIR_file_content, "NOMPRES"); // Nominal inflation pressure

    // VERTICAL
    FNOMIN = get_coefficient(TIR_file_content, "FNOMIN"); // Nominal (rated) wheel load

    // SCALING_COEFFICIENTS
    LFZO = get_coefficient(TIR_file_content, "LFZO");
    LCX = get_coefficient(TIR_file_content, "LCX");
    LMUX = get_coefficient(TIR_file_content, "LMUX");
    LEX = get_coefficient(TIR_file_content, "LEX");
    LKX = get_coefficient(TIR_file_content, "LKX");
    LHX = get_coefficient(TIR_file_content, "LHX");
    LVX = get_coefficient(TIR_file_content, "LVX");
    LCY = get_coefficient(TIR_file_content, "LCY");
    LMUY = get_coefficient(TIR_file_content, "LMUY");
    LEY = get_coefficient(TIR_file_content, "LEY");
    LKY = get_coefficient(TIR_file_content, "LKY");
    LKYC = get_coefficient(TIR_file_content, "LKYC");
    LKZC = get_coefficient(TIR_file_content, "LKZC");
    LHY = get_coefficient(TIR_file_content, "LHY");
    LVY = get_coefficient(TIR_file_content, "LVY");
    LTR = get_coefficient(TIR_file_content, "LTR");
    LRES = get_coefficient(TIR_file_content, "LRES");
    LXAL = get_coefficient(TIR_file_content, "LXAL");
    LYKA = get_coefficient(TIR_file_content, "LYKA");
    LVYKA = get_coefficient(TIR_file_content, "LVYKA");
    LS = get_coefficient(TIR_file_content, "LS");
    LMX = get_coefficient(TIR_file_content, "LMX");
    LVMX = get_coefficient(TIR_file_content, "LVMX");
    LMY = get_coefficient(TIR_file_content, "LMY");
    LMP = get_coefficient(TIR_file_content, "LMP");

    // LONGITUDINAL_COEFFICIENTS
    PCX1 = get_coefficient(TIR_file_content, "PCX1");
    PDX1 = get_coefficient(TIR_file_content, "PDX1");
    PDX2 = get_coefficient(TIR_file_content, "PDX2");
    PDX3 = get_coefficient(TIR_file_content, "PDX3");
    PEX1 = get_coefficient(TIR_file_content, "PEX1");
    PEX2 = get_coefficient(TIR_file_content, "PEX2");
    PEX3 = get_coefficient(TIR_file_content, "PEX3");
    PEX4 = get_coefficient(TIR_file_content, "PEX4");
    PKX1 = get_coefficient(TIR_file_content, "PKX1");
    PKX2 = get_coefficient(TIR_file_content, "PKX2");
    PKX3 = get_coefficient(TIR_file_content, "PKX3");
    PHX1 = get_coefficient(TIR_file_content, "PHX1");
    PHX2 = get_coefficient(TIR_file_content, "PHX2");
    PVX1 = get_coefficient(TIR_file_content, "PVX1");
    PVX2 = get_coefficient(TIR_file_content, "PVX2");
    RBX1 = get_coefficient(TIR_file_content, "RBX1");
    RBX2 = get_coefficient(TIR_file_content, "RBX2");
    RBX3 = get_coefficient(TIR_file_content, "RBX3");
    RCX1 = get_coefficient(TIR_file_content, "RCX1");
    REX1 = get_coefficient(TIR_file_content, "REX1");
    REX2 = get_coefficient(TIR_file_content, "REX2");
    RHX1 = get_coefficient(TIR_file_content, "RHX1");
    PPX1 = get_coefficient(TIR_file_content, "PPX1");
    PPX2 = get_coefficient(TIR_file_content, "PPX2");
    PPX3 = get_coefficient(TIR_file_content, "PPX3");
    PPX4 = get_coefficient(TIR_file_content, "PPX4");

    // OVERTURNING_COEFFICIENTS
    QSX1 = get_coefficient(TIR_file_content, "QSX1");
    QSX2 = get_coefficient(TIR_file_content, "QSX2");
    QSX3 = get_coefficient(TIR_file_content, "QSX3");
    QSX4 = get_coefficient(TIR_file_content, "QSX4");
    QSX5 = get_coefficient(TIR_file_content, "QSX5");
    QSX6 = get_coefficient(TIR_file_content, "QSX6");
    QSX7 = get_coefficient(TIR_file_content, "QSX7");
    QSX8 = get_coefficient(TIR_file_content, "QSX8");
    QSX9 = get_coefficient(TIR_file_content, "QSX9");
    QSX10 = get_coefficient(TIR_file_content, "QSX10");
    QSX11 = get_coefficient(TIR_file_content, "QSX11");
    QSX12 = get_coefficient(TIR_file_content, "QSX12");
    QSX13 = get_coefficient(TIR_file_content, "QSX13");
    QSX14 = get_coefficient(TIR_file_content, "QSX14");
    PPMX1 = get_coefficient(TIR_file_content, "PPMX1");

    // LATERAL_COEFFICIENTS
    PCY1 = get_coefficient(TIR_file_content, "PCY1");
    PDY1 = get_coefficient(TIR_file_content, "PDY1");
    PDY2 = get_coefficient(TIR_file_content, "PDY2");
    PDY3 = get_coefficient(TIR_file_content, "PDY3");
    PEY1 = get_coefficient(TIR_file_content, "PEY1");
    PEY2 = get_coefficient(TIR_file_content, "PEY2");
    PEY3 = get_coefficient(TIR_file_content, "PEY3");
    PEY4 = get_coefficient(TIR_file_content, "PEY4");
    PEY5 = get_coefficient(TIR_file_content, "PEY5");
    PKY1 = get_coefficient(TIR_file_content, "PKY1");
    PKY2 = get_coefficient(TIR_file_content, "PKY2");
    PKY3 = get_coefficient(TIR_file_content, "PKY3");
    PKY4 = get_coefficient(TIR_file_content, "PKY4");
    PKY5 = get_coefficient(TIR_file_content, "PKY5");
    PKY6 = get_coefficient(TIR_file_content, "PKY6");
    PKY7 = get_coefficient(TIR_file_content, "PKY7");
    PHY1 = get_coefficient(TIR_file_content, "PHY1");
    PHY2 = get_coefficient(TIR_file_content, "PHY2");
    PVY1 = get_coefficient(TIR_file_content, "PVY1");
    PVY2 = get_coefficient(TIR_file_content, "PVY2");
    PVY3 = get_coefficient(TIR_file_content, "PVY3");
    PVY4 = get_coefficient(TIR_file_content, "PVY4");
    RBY1 = get_coefficient(TIR_file_content, "RBY1");
    RBY2 = get_coefficient(TIR_file_content, "RBY2");
    RBY3 = get_coefficient(TIR_file_content, "RBY3");
    RBY4 = get_coefficient(TIR_file_content, "RBY4");
    RCY1 = get_coefficient(TIR_file_content, "RCY1");
    REY1 = get_coefficient(TIR_file_content, "REY1");
    REY2 = get_coefficient(TIR_file_content, "REY2");
    RHY1 = get_coefficient(TIR_file_content, "RHY1");
    RHY2 = get_coefficient(TIR_file_content, "RHY2");
    RVY1 = get_coefficient(TIR_file_content, "RVY1");
    RVY2 = get_coefficient(TIR_file_content, "RVY2");
    RVY3 = get_coefficient(TIR_file_content, "RVY3");
    RVY4 = get_coefficient(TIR_file_content, "RVY4");
    RVY5 = get_coefficient(TIR_file_content, "RVY5");
    RVY6 = get_coefficient(TIR_file_content, "RVY6");
    PPY1 = get_coefficient(TIR_file_content, "PPY1");
    PPY2 = get_coefficient(TIR_file_content, "PPY2");
    PPY3 = get_coefficient(TIR_file_content, "PPY3");
    PPY4 = get_coefficient(TIR_file_content, "PPY4");
    PPY5 = get_coefficient(TIR_file_content, "PPY5");

    // ALIGNING_COEFFICIENTS
    QBZ1 = get_coefficient(TIR_file_content, "QBZ1");
    QBZ2 = get_coefficient(TIR_file_content, "QBZ2");
    QBZ3 = get_coefficient(TIR_file_content, "QBZ3");
    QBZ4 = get_coefficient(TIR_file_content, "QBZ4");
    QBZ5 = get_coefficient(TIR_file_content, "QBZ5");
    QBZ9 = get_coefficient(TIR_file_content, "QBZ9");
    QBZ10 = get_coefficient(TIR_file_content, "QBZ10");
    QCZ1 = get_coefficient(TIR_file_content, "QCZ1");
    QDZ1 = get_coefficient(TIR_file_content, "QDZ1");
    QDZ2 = get_coefficient(TIR_file_content, "QDZ2");
    QDZ3 = get_coefficient(TIR_file_content, "QDZ3");
    QDZ4 = get_coefficient(TIR_file_content, "QDZ4");
    QDZ6 = get_coefficient(TIR_file_content, "QDZ6");
    QDZ7 = get_coefficient(TIR_file_content, "QDZ7");
    QDZ8 = get_coefficient(TIR_file_content, "QDZ8");
    QDZ9 = get_coefficient(TIR_file_content, "QDZ9");
    QDZ10 = get_coefficient(TIR_file_content, "QDZ10");
    QDZ11 = get_coefficient(TIR_file_content, "QDZ11");
    QEZ1 = get_coefficient(TIR_file_content, "QEZ1");
    QEZ2 = get_coefficient(TIR_file_content, "QEZ2");
    QEZ3 = get_coefficient(TIR_file_content, "QEZ3");
    QEZ4 = get_coefficient(TIR_file_content, "QEZ4");
    QEZ5 = get_coefficient(TIR_file_content, "QEZ5");
    QHZ1 = get_coefficient(TIR_file_content, "QHZ1");
    QHZ2 = get_coefficient(TIR_file_content, "QHZ2");
    QHZ3 = get_coefficient(TIR_file_content, "QHZ3");
    QHZ4 = get_coefficient(TIR_file_content, "QHZ4");
    SSZ1 = get_coefficient(TIR_file_content, "SSZ1");
    SSZ2 = get_coefficient(TIR_file_content, "SSZ2");
    SSZ3 = get_coefficient(TIR_file_content, "SSZ3");
    SSZ4 = get_coefficient(TIR_file_content, "SSZ4");
    PPZ1 = get_coefficient(TIR_file_content, "PPZ1");
    PPZ2 = get_coefficient(TIR_file_content, "PPZ2");

    // ROLLING_COEFFICIENTS
    QSY1 = get_coefficient(TIR_file_content, "QSY1");
    QSY2 = get_coefficient(TIR_file_content, "QSY2");
    QSY3 = get_coefficient(TIR_file_content, "QSY3");
    QSY4 = get_coefficient(TIR_file_content, "QSY4");
    QSY5 = get_coefficient(TIR_file_content, "QSY5");
    QSY6 = get_coefficient(TIR_file_content, "QSY6");
    QSY7 = get_coefficient(TIR_file_content, "QSY7");
    QSY8 = get_coefficient(TIR_file_content, "QSY8");
}

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