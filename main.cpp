#include <iostream>
#include "MF_tire_model.h"

int main() {

    double MF_output [5] {}; // [Fx, Fy, My, Mz, Mx] // use std::array
    MF_tire_model tire{"../tir_file.txt"};

    return 0;
}
