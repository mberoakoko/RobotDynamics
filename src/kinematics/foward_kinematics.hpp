//
// Created by mbero on 26/05/2025.
//

#ifndef FOWARD_KINEMATICS_HPP
#define FOWARD_KINEMATICS_HPP
#include <ostream>
#include <bits/stdc++.h>
#include "../kinematics/kinetics.hpp"
#include <eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>


namespace ForwardKinematics{
    struct Bound{
        std::size_t bound;
        bool operator==(const std::size_t x) const { return x == bound; }
    };

    inline auto forward_kinematics_in_body(const Eigen::Matrix4f& M, const std::vector<Kinetics::Vector6>& b_list, std::vector<float> theta_list) -> Eigen::Matrix4f{
        assert(b_list.size() == theta_list.size());
        std::size_t n = b_list.size();
        auto screws = std::views::iota(0, Bound{n})
        | std::views::transform([&theta_list, &b_list](auto x){return theta_list.at(x) * b_list.at(x);})
        | std::views::transform([](const Kinetics::Vector6& b_screw){return Kinetics::matrix_exponent_6(b_screw);});
        Eigen::Matrix4f cache = Eigen::Matrix4f::Identity();
        for (const auto item : screws) {
            cache = cache * item;
        }
        Eigen::Matrix4f result = M * cache;
        return result;
    };
    inline auto forward_kinematics_in_space(const Eigen::Matrix4f& M, const std::vector<Kinetics::Vector6>& s_list, std::vector<float> theta_list) -> Eigen::Matrix4f {
        assert(s_list.size() == theta_list.size());
        std::size_t n = s_list.size();
        auto screws = std::views::iota(0, Bound{n})
        | std::views::transform([&theta_list, &s_list](auto x){return theta_list.at(x) * s_list.at(x);})
        | std::views::transform([](const Kinetics::Vector6& b_screw){return Kinetics::matrix_exponent_6(b_screw);});
        Eigen::Matrix4f cache = Eigen::Matrix4f::Identity();
        for (const auto item : screws) {
            cache = cache * item;
        }
        Eigen::Matrix4f result = cache * M;
        return result;
    }
}
#endif //FOWARD_KINEMATICS_HPP
