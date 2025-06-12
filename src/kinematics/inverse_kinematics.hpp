//
// Created by mbero on 26/05/2025.
//

#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP
#include <ostream>
#include <bits/stdc++.h>
#include "../kinematics/kinetics.hpp"
#include <eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

namespace InverseKinematics {
    using JacobianFunction = std::function<Eigen::MatrixXf(Eigen::VectorXd)>;

    using CartesianFowardKinematicsFunction = std::function<Eigen::Vector3f(Eigen::VectorXd)>;
    using TwistFowardKinematicsFunction = std::function<Kinetics::Vector6(Eigen::VectorXd)>;

    /**
     * The function calulates the  joint position given the desired cartesian coordinates
     * @tparam N Number of robot joints
     * @param f foward kinematics function
     * @param j Jacobian Function
     * @param x_d desired robot position
     * @param epsilon threshold
     * @param initial_guess initial guess of robot joint positions
     * @return robot joint postions
     */
    template<std::size_t N>
    inline auto inverse_kinematics_cartesian(
        const CartesianFowardKinematicsFunction&& f,
        const JacobianFunction&& j,
        const Eigen::Vector3f& x_d,
        double epsilon,
        const std::optional<Eigen::VectorXd>& initial_guess = {}) -> Eigen::VectorXd {
        auto theta_star = initial_guess.value_or(Eigen::VectorXd::Random(N)); // this is bug, I dont know how big the size of theta is

        while ((x_d - f(theta_star)).norm() > epsilon) {
            theta_star = theta_star + j(theta_star).completeOrthogonalDecomposition().pseudoInverse();
        }
        return theta_star;
    }

    template<std::size_t N>
    inline auto inverse_kinematics_screw(
        const TwistFowardKinematicsFunction&& f,
        const JacobianFunction&& j,
        const Eigen::VectorXd& x_d,
        double epsilon,
        const std::optional<Eigen::VectorXd>& initial_guess = {}
        ) -> Eigen::VectorXd {
        //TODO: I need to implement the matrix log before I continue
    }
}
#endif //INVERSE_KINEMATICS_HPP
