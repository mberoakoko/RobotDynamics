cmake_minimum_required(VERSION 3.22.1)
project(RobotDynamics)

set(CMAKE_CXX_STANDARD 23)


add_executable(RobotDynamics main.cpp
        src/kinematics/kinetics.hpp
        src/kinematics/foward_kinematics.hpp
        src/kinematics/velocity_kinematics.hpp
        src/kinematics/inverse_kinematics.hpp)
