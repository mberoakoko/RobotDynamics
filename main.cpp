#include <iostream>
#include "src/kinematics/kinetics.hpp"

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

int main() {
    auto [omega_3_hat, theta] { Kinetics::axis_angle_3({1, 2, 3}) };
    std::cout<<"Theta " <<theta<<"\n";
    std::cout<<"Axis\n" <<omega_3_hat << "\n";
    std::cout << Kinetics::vec_3_to_so3(omega_3_hat) << std::endl;
    auto temp = Kinetics::vec_3_to_so3(omega_3_hat);
    std::cout << temp * temp << std::endl;
    auto rotated_matrix = Kinetics::rot_axis(omega_3_hat, theta);
    std::cout<<"RotatedMatrix\n" <<rotated_matrix << "\n";

    return 0;
}