//
// Created by mbero on 26/05/2025.
//

#ifndef KINETICS_HPP
#define KINETICS_HPP

#include <ostream>
#include <bits/stdc++.h>
#include <eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

namespace Kinetics {
    using Screw = Eigen::Vector<float, 6>;
    using Vector6 = Eigen::Vector<float, 6>;
    using AdjointMatrix = Eigen::Matrix<float, 6, 6>;
    struct AxisAngle6 {
        Screw screw;
        float theta;

        auto to_vector() const -> Vector6 { return theta * screw; };

        static auto from_vector(const Vector6& vector) -> AxisAngle6 {
            return {
                .screw = vector.normalized(),
                .theta = vector.norm(),
            };
        }
    };

    struct LittleSo3Coordinates {
        Eigen::Vector3f omega_hat;
        float theta;

        friend std::ostream & operator<<(std::ostream &os, const LittleSo3Coordinates &obj) {
            return os
                    <<"Little So3 coordinates:\n"
                    << "omega_hat: " << obj.omega_hat.transpose()
                    << " theta: " << obj.theta;
        }
    };
    struct ScrewParams {
        const Eigen::Vector3f& q; // Displacement
        const Eigen::Vector3f& omega_hat;
        const float& h { 0 }; // Pitch
    };;




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

        friend std::ostream & operator<<(std::ostream &os, const ExponentialCoordinate &obj) {
            return os
                    << "Exponential Coordinates \n"
                   << "omega_3_hat:\n" << obj.omega_3_hat.transpose() << "\n"
                   << " theta: " << obj.theta;
        }
    };

    inline auto to_screw(const ScrewParams& screw_params) -> Screw {
        Screw result = Screw::Zero(6);
        result.block(0, 0, 3, 1) = screw_params.omega_hat;
        Eigen::Vector3f velocity = -screw_params.q.cross(screw_params.omega_hat) + (screw_params.h * screw_params.omega_hat);
        result.block(3, 0, 3, 1 ) = velocity;
        return result;
    }

    inline auto to_screw(const Eigen::Vector3f& q, const Eigen::Vector3f& omega_hat, float pitch) -> Screw {
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
        return Eigen::Vector3f{-so3(1, 2), so3(0, 2), -so3(0, 1)};
    }

    inline auto axis_angle_3(const Eigen::Vector3f & exponential_3) -> ExponentialCoordinate {
        return ExponentialCoordinate::from_vector(exponential_3);
    }

    inline auto matrix_exponent_3(const Eigen::Matrix3f& so3_matrix) -> Eigen::Matrix3f {
        Eigen::Vector3f exponential_3 = Kinetics::so3_to_vec_3(so3_matrix);
        auto [omega_3_hat, theta] = Kinetics::axis_angle_3(exponential_3);
        const Eigen::Matrix3f omega_skew = Kinetics::vec_3_to_so3(omega_3_hat);
        const auto expr_1 = ::sin(theta) * omega_skew;
        const auto expr_2 = (1 - ::cos(theta)) * (omega_skew * omega_skew);
        return Eigen::Matrix3f::Identity() + expr_1 + expr_2;
    }

    inline auto rot_axis(const Eigen::Vector3f& omega , const float theta) -> Eigen::Matrix3f {
        return Kinetics::matrix_exponent_3(
            Kinetics::vec_3_to_so3(omega * theta)
        );
    }


    inline auto get_omega_from_rotation_matrix_(const Eigen::Matrix3f& rotation_matrix) -> Eigen::Vector3f {
        float r_1_3 = rotation_matrix(0, 2);
        float r_2_3 = rotation_matrix(1, 2);
        float r_3_3 = rotation_matrix(2, 2);
        return (1 / ::sqrt(2 * (1 + r_3_3))) * Eigen::Vector3f({r_1_3, r_2_3, r_3_3});
    }


    inline auto matrix_log_3(const Eigen::MatrixX3f& rotation_matrix) -> LittleSo3Coordinates {
        if (rotation_matrix.isApprox(Eigen::Matrix3f::Identity())) {
            return {.omega_hat = {}, .theta = 0};
        }
        auto omega_hat = get_omega_from_rotation_matrix_(rotation_matrix);
        if (rotation_matrix.trace() == -1) {
            return {
                .omega_hat = omega_hat,
                .theta = 0
            };
        }
        const float theta = ::acos(0.5*(rotation_matrix.trace() - 1));
        const Eigen::Vector3f omega_3_hat = so3_to_vec_3((1/(2*::sin(theta))) * (rotation_matrix - rotation_matrix.transpose()));
        return {
            .omega_hat = omega_3_hat,
            .theta = theta
        };
    }

    inline auto r_p_to_trans(const Eigen::Matrix3f& r, const Eigen::Vector3f& p) -> Eigen::Matrix4f {
        Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
        Eigen::Matrix3f rotation_matrix = r;
        result.block(0, 0, 3, 3) = rotation_matrix;
        result.block(0, 3, 3, 1) = p;
        result(3, 3) = 1;
        return result;
    }

    inline auto trans_inverse(const Eigen::Matrix4f& trans_mat) -> Eigen::Matrix4f {
        Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
        Eigen::Matrix3f rotation_mat = Kinetics::rotation_inverse(trans_mat.block(0, 0, 3, 3));
        Eigen::Vector3f translation = trans_mat.block(0, 3, 3, 1);
        result.block(0, 0, 3, 3) = rotation_mat;
        result.block(0, 3, 3, 1) = rotation_mat * translation;
        return result;
    }

    inline auto vec_to_se3(const Vector6& screw_vector) -> Eigen::Matrix4f {
        Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
        Eigen::Matrix3f omega_skew = Kinetics::vec_3_to_so3(screw_vector.head(3));
        Eigen::Vector3f velocity = screw_vector.tail(3);
        result.block(0, 0, 3, 3) = omega_skew;
        result.block(0, 3, 3, 1) = velocity;
        return result;
    }

    inline auto se3_to_vec(const Eigen::Matrix4f& se3_matrix) -> Vector6 {
        Vector6 result;
        Eigen::Vector3f omega_hat = Kinetics::so3_to_vec_3(se3_matrix.block(0, 0, 3, 3));
        Eigen::Vector3f velocity = se3_matrix.block(0, 3, 3, 1);
        result.head(3) = omega_hat;
        result.tail(3) = velocity;
        return result;
    }

    inline auto to_adjoint(const Eigen::Matrix4f& trans_mat) {
        AdjointMatrix adjoint = AdjointMatrix::Zero();
        Eigen::Matrix3f rotation_matrix = trans_mat.block(0, 0, 3, 3);
        Eigen::Vector3f translation = trans_mat.block(0, 3, 3, 1);
        adjoint.block(0, 0, 3, 3) = rotation_matrix;
        adjoint.block(0, 3, 3, 3) = Eigen::Matrix3f::Zero();
        adjoint.block(3, 0, 3, 3) = Kinetics::vec_3_to_so3(translation) * rotation_matrix;
        adjoint.block(3, 3, 3, 3) = rotation_matrix;
        return adjoint;
    }

    inline auto to_axis_angle_6(const Vector6& exponential_coordinates) -> AxisAngle6 {
        return AxisAngle6::from_vector(exponential_coordinates);
    }

    inline auto matrix_exponent_6(const Vector6& exponential_coodinates) -> Eigen::Matrix4f {
        Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
        const auto [screw , theta ] = to_axis_angle_6(exponential_coodinates);
        Eigen::Matrix3f omega_skew = Kinetics::vec_3_to_so3(screw.head(3));
        Eigen::Vector3f velocity = screw.tail(3);
        result.block(0, 0, 3, 3) = Kinetics::matrix_exponent_3(omega_skew);
        auto expr1 = Eigen::Matrix3f::Identity() * theta;
        auto expr2 = (1 - ::cos(theta)) * omega_skew;
        auto expr3 = (theta - ::sin(theta)) * (omega_skew * omega_skew) ;
        Eigen::Vector3f displacement = (expr1 + expr2 + expr3) * velocity;
        result.block(0, 3, 3, 1) = displacement;
        result.block(3, 3, 1, 1) = Eigen::Matrix<float, 1, 1, 1>{{1}};
        return result;
    }

    inline auto matrix_exponent_6(const Vector6& screw , double theta) -> Eigen::Matrix4f {
        return matrix_exponent_6(screw * theta);
    }
}
#endif //KINETICS_HPP
