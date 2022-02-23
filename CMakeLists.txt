cmake_minimum_required(VERSION 3.21)
project(tire_model)

set(CMAKE_CXX_STANDARD 17)

add_executable(tire_model main.cpp MF_tire_model.cpp MF_tire_model.h)
