//
// Created by mbero on 26/05/2025.
//

#ifndef KINETICS_HPP
#define KINETICS_HPP

#include <bits/stdc++.h>
#include <eigen3/Eigen/Core>
#include <oneapi/tbb/task_arena.h>

namespace Kinetics {
    using Screw = Eigen::Vector<float, 6>;
    struct Twist {
        Screw screw;
        float theta;
    };
    struct ScrewParams {
        float q; // Displacement
        Eigen::Vector3f omega_hat;
        float h; // Pitch
    };




    struct ExponentialCoordinate {
        Eigen::Vector3f omega_3_hat;
        float theta;

        auto to_vector() const -> Eigen::Vector3f { return theta * omega_3_hat; }
        static auto from_vector(const Eigen::Vector3f &v) -> ExponentialCoordinate {
            return {
                .omega_3_hat = v.normalized(),
                .theta = v.norm()
            };
        }
    };

    inline auto to_screw(const ScrewParams& screw_params) -> Screw {
        Screw result = Screw::Zero(6);
        result.block(0, 0, 3, 1) = screw_params.omega_hat;
        return result;
    }

    inline auto to_screw(float q, const Eigen::Vector3f& omega_hat, float pitch) -> Screw {
        return to_screw({
            .q = q,
            .omega_hat = omega_hat,
            .h = pitch
        });
    }



    inline auto rotation_inverse(const Eigen::Matrix3f& rotation_matrix) -> Eigen::Matrix3f {
        Eigen::Matrix3f inverse = rotation_matrix.transpose();
        return inverse;
    };

    inline auto vec_3_to_so3(const Eigen::Vector3f& vector) -> Eigen::Matrix3f {
        return Eigen::Matrix3f{
            {
                {0              , -vector(2),   vector(1)},
                {vector(2),                 0, -vector(0)},
                {-vector(1), vector(0),                 0}
            }
            };
    }

    inline auto so3_to_vec_3(const Eigen::Matrix3f& so3) -> Eigen::Vector3f {
        return Eigen::Vector3f{-so3(0, 1), so3(0, 2), -so3(1, 2)};
    }

    inline auto axis_angle_3(const Eigen::Vector3f & exponential_3) -> ExponentialCoordinate {
        return ExponentialCoordinate::from_vector(exponential_3);
    }

    inline auto matrix_exponent_3(const Eigen::Matrix3f& so3_matrix) -> Eigen::Matrix3f {
        auto exponential_3 = Kinetics::so3_to_vec_3(so3_matrix);
        auto [omega_3_hat, theta] = Kinetics::axis_angle_3(exponential_3);
        const auto omega_skew = Kinetics::vec_3_to_so3(omega_3_hat);
        const auto expr_1 = ::sin(theta) * omega_skew;
        const auto expr_2 = (1 - ::cos(theta)) * (omega_skew * omega_skew);
        return Eigen::Matrix3f::Identity() + expr_1 * expr_2;
    }

    inline auto rot_axis(const Eigen::Vector3f& omega , const float theta) -> Eigen::Matrix3f {
        return Kinetics::matrix_exponent_3(
            Kinetics::vec_3_to_so3(omega * theta)
        );
    }

    inline auto matrix_log_3(const Eigen::MatrixX3f& rotation_matrix) -> Twist {
        return {};
    }

    inline auto r_p_to_trans(const Eigen::Matrix3f& r, const Eigen::Vector3f& p) -> Eigen::Matrix4f {
        Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
        Eigen::Matrix3f rotation_matrix = r;
        result.block(0, 0, 3, 3) = rotation_matrix;
        result.block(0, 3, 3, 1) = p;
        result(3, 3) = 1;
        return result;
    }

}
#endif //KINETICS_HPP
