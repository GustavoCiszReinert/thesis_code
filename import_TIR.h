//
// Created by gusta on 14/06/2022.
//

#ifndef TIRE_MODEL_IMPORT_TIR_H
#define TIRE_MODEL_IMPORT_TIR_H

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>

void import_TIR_file (std::string file_name, std::vector<std::string> &content){
    std::ifstream in_file {file_name};
    std::string word {};
    if (!in_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
    }
    while (in_file >> word) {
        content.push_back(word);
    }
    in_file.close();
}

double get_coefficient (std::vector<std::string> TIR_file_content, const std::string& parameter_name){
    double variable_value {};
    auto loc = std::find(TIR_file_content.begin(), TIR_file_content.end(), parameter_name);
    if (loc != TIR_file_content.end()){
        variable_value = std::stod(*(loc + 2));
    } else {
        std::cerr << parameter_name << " value has not been found" << std::endl;
    }
    return variable_value;
}

#endif //TIRE_MODEL_IMPORT_TIR_H
