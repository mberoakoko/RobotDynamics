#include <iostream>
#include <limits>

#include "src/kinematics/kinetics.hpp"
#include<eigen3/Eigen/Geometry>

#include "src/kinematics/foward_kinematics.hpp"

class from_string {
    const char* str_;
public:
    explicit from_string(const char* str): str_(str) {}

    template<typename type>
    explicit operator type() {
        if constexpr (std::is_same_v<type, float>) {return std::stof(str_);}
        else if constexpr (std::is_same_v<type, double>) {return std::stod(str_);}
        return std::stoi(str_);
    }
};


auto run_experiments() -> void {
    Eigen::Matrix4f home = Eigen::Matrix4f::Identity();
    std::vector<Kinetics::Vector6> b_list{
        Kinetics::Vector6{1, 2, 3, 1,2 , 3},
        Kinetics::Vector6{1, 2, 3, 1, 2, 3},
        Kinetics::Vector6{1, 2, 31, 1, 2, 3}};
    std::vector<float> theta_list {M_PI, 0, 0};
    std::cout << ForwardKinematics::forward_kinematics_in_body(home, b_list, theta_list);
}


int main() {
    run_experiments();
    return 0;
}