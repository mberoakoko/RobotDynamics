//
// Created by mbero on 26/05/2025.
//

#ifndef VELOCITY_KINEMATICS_HPP
#define VELOCITY_KINEMATICS_HPP
#include <ostream>
#include <bits/stdc++.h>
#include "../kinematics/kinetics.hpp"
#include <eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
namespace VelocityKinematics {
    struct Bound {
        std::size_t bound;
        bool operator==(const std::size_t x) const {return x == bound;};
    };


    class TransformationAccumulator {
    public:
        struct zipped_pair {
            Eigen::Matrix4f mat;
            Kinetics::Vector6 vec;
        };
        explicit TransformationAccumulator(const Kinetics::Vector6& s_1) {
            jacobian_cache_.push_back(s_1);
        };
        friend auto operator +(TransformationAccumulator acc, const zipped_pair& pair) -> TransformationAccumulator {
            acc.accum_mat_ = acc.accum_mat_ * pair.mat;
            acc.jacobian_cache_.emplace_back(Kinetics::to_adjoint(acc.accum_mat_) * pair.vec );
            return acc;
        };

        [[nodiscard]]
        auto jacobian_entries()const -> std::vector<Kinetics::Vector6> {
            return jacobian_cache_;
        }
    private:
        Eigen::Matrix4f accum_mat_ = Eigen::Matrix4f::Identity();
        std::vector<Kinetics::Vector6> jacobian_cache_;
    };

    namespace Utils {

        inline auto get_homogenous_matrix_list(const std::vector<Kinetics::Vector6>& screw_list, std::vector<float> theta_list) -> std::vector<Eigen::Matrix4f> {
            auto to_twist = [&theta_list, &screw_list](const std::size_t index) {
                return screw_list.at(index) * theta_list.at(index);
            };
            const auto to_homogenous_mat = [](const Kinetics::Vector6& screw_vec) {
                return Kinetics::matrix_exponent_6(-screw_vec);
            };
            const auto homogenous_mat_series = std::views::iota(0, VelocityKinematics::Bound(screw_list.size() - 1))
                    | std::views::transform(to_twist)
                    | std::views::transform(to_homogenous_mat);
            std::vector<Eigen::Matrix4f> result;
            std::ranges::copy(homogenous_mat_series, std::back_inserter(result));
            return result;
        }

        inline auto generate_zipped_pair(
                        const std::vector<Eigen::Matrix4f>& matrices,
                        const std::vector<Kinetics::Vector6>& screw_list) -> std::vector<TransformationAccumulator::zipped_pair> {
            const auto to_zipped_pair = [](const Eigen::MatrixX4f& homogenous_mat , const Kinetics::Vector6& vector) -> TransformationAccumulator::zipped_pair {
                return {
                    .mat = homogenous_mat,
                    .vec = vector
                };
            };
            auto zipped_pairs_range = std::views::iota(0, Bound(screw_list.size() - 1))
            | std::views::transform([&matrices, &screw_list, &to_zipped_pair](const std::size_t index) {
                return to_zipped_pair(matrices.at(index), screw_list.at(index));
            });
            std::vector<TransformationAccumulator::zipped_pair> result;
            std::ranges::copy(zipped_pairs_range, std::back_inserter(result));
            return  result;
        }
    }

    inline auto jacobian_in_body(const std::vector<Kinetics::Vector6>& b_list, std::vector<float> theta_list) -> Eigen::MatrixXf {
        assert(b_list.size() - 1 == theta_list.size());
        const auto to_twist = [&theta_list, &b_list](const std::size_t index) {
            std::cout << " Index " << index << std::endl;
            return theta_list.at(index) * b_list.at(index);
        };
        const auto to_homogenous_mat = [](const Kinetics::Vector6& screw_vec) {
            return Kinetics::matrix_exponent_6(-screw_vec);
        };

        const auto to_zipped_pair = [](const Eigen::MatrixX4f& homogenous_mat , const Kinetics::Vector6& vector) -> TransformationAccumulator::zipped_pair {
            return {
                .mat = homogenous_mat,
                .vec = vector
            };
        };
        const auto homogenous_mat_series = std::views::iota(0, Bound(b_list.size() - 1))
                | std::views::transform(to_twist)
                | std::views::transform(to_homogenous_mat);


        std::vector<Eigen::Matrix4f> matrices;
        std::ranges::copy(homogenous_mat_series, std::back_inserter(matrices));

        auto zipped_pairs_range = std::views::iota(0, Bound(b_list.size() - 1))
            | std::views::transform([&matrices, &b_list, &to_zipped_pair](const std::size_t index) {
                return to_zipped_pair(matrices.at(index), b_list.at(index));
            });
        std::vector<TransformationAccumulator::zipped_pair> zipped_pairs;
        std::ranges::copy(zipped_pairs_range, std::back_inserter(zipped_pairs));
        TransformationAccumulator trans_acc{b_list.at(0)};
        auto transformed_acc = std::accumulate(std::begin(zipped_pairs), std::end(zipped_pairs), trans_acc);
        Eigen::MatrixXf jacobian(6, transformed_acc.jacobian_entries().size());
        for (int index = 0; index < transformed_acc.jacobian_entries().size(); ++index) {
            jacobian.col(index) = transformed_acc.jacobian_entries().at(index);
        }
        return jacobian;

    }
    inline auto jacobian_in_space(const std::vector<Kinetics::Vector6>& s_list,const std::vector<float>& theta_list) -> Eigen::MatrixXd {
        assert(s_list.size() - 1 == theta_list.size());
        auto matrices { VelocityKinematics::Utils::get_homogenous_matrix_list(s_list, theta_list) };
        auto zipped_pairs{Utils::generate_zipped_pair(matrices, s_list)};
        auto trans_acc = std::accumulate(
            std::begin(zipped_pairs), std::end(zipped_pairs),
            TransformationAccumulator{s_list.at(0)}
            );
        Eigen::MatrixXf jacobian(6, trans_acc.jacobian_entries().size());
        for (int index = 1; index < trans_acc.jacobian_entries().size(); ++index) {
            jacobian.col(index) = trans_acc.jacobian_entries().at(index);
        }
        return jacobian;
    }
}
#endif //VELOCITY_KINEMATICS_HPP
