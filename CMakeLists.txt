cmake_minimum_required(VERSION 3.22.1)
project(RobotDynamics)

find_package(Pangolin REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 23)


add_executable(RobotDynamics main.cpp
        src/kinematics/kinetics.hpp
        src/kinematics/foward_kinematics.hpp
        src/kinematics/velocity_kinematics.hpp
        src/kinematics/inverse_kinematics.hpp
        src/graphing/graph_primitives.hpp
        src/utils/iterations.hpp
        src/aether_rendering_engine/core_rendering_demo.hpp)
# Pangolin's legacy variables require you to explicitly provide header paths
target_include_directories(RobotDynamics PRIVATE ${Pangolin_INCLUDE_DIRS})

target_link_libraries(RobotDynamics
        PRIVATE
        ${Pangolin_LIBRARIES}
        Eigen3::Eigen
)
