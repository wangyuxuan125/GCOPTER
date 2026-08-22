/*
    MIT License

    Copyright (c) 2026 TF-SFC contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef TRAJ_FAVORABLE_SFC_HPP
#define TRAJ_FAVORABLE_SFC_HPP

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

namespace tf_sfc
{

    enum class DirectionMode
    {
        FRENET = 0,
        PCA = 1,
        SENSITIVITY = 2
    };

    struct Parameters
    {
        int max_faces = 12;
        int max_obs_faces = 6; // Reserved for the obstacle-cutting extension.
        double safety_margin = 0.25;
        double min_overlap_radius = 0.15;
        double max_inflation_distance = 1.0;
        double inflation_step = 0.10;
        double tangent_weight = 1.0;
        double lateral_weight = 0.5;
        DirectionMode direction_mode = DirectionMode::PCA;
    };

    struct Corridor
    {
        // GCOPTER convention: each row stores [n_x, n_y, n_z, d] for
        // n.dot(x) + d <= 0.
        Eigen::MatrixX4d hpoly;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d utility = Eigen::Vector3d::Ones();
        int piece_id = -1;
        int face_num = 0;
        double generation_time_ms = 0.0;
        double weighted_width = 0.0;
        double min_sample_slack = -std::numeric_limits<double>::infinity();
        bool valid = false;
        bool direction_fallback = false;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    namespace detail
    {
        constexpr double kDirectionEpsilon = 1.0e-9;

        inline bool makeRightHandedFrame(const Eigen::Vector3d &primary,
                                         const Eigen::Vector3d &secondaryHint,
                                         Eigen::Matrix3d &frame)
        {
            if (primary.norm() < kDirectionEpsilon)
            {
                return false;
            }

            Eigen::Vector3d e0 = primary.normalized();
            Eigen::Vector3d e1 = secondaryHint - e0 * e0.dot(secondaryHint);
            if (e1.norm() < kDirectionEpsilon)
            {
                const Eigen::Vector3d helper = std::abs(e0.z()) < 0.9
                                                   ? Eigen::Vector3d::UnitZ()
                                                   : Eigen::Vector3d::UnitY();
                e1 = helper - e0 * e0.dot(helper);
            }
            if (e1.norm() < kDirectionEpsilon)
            {
                return false;
            }

            e1.normalize();
            frame.col(0) = e0;
            frame.col(1) = e1;
            frame.col(2) = e0.cross(e1).normalized();
            return true;
        }

        inline bool eigenDirections(const Eigen::Matrix3d &matrix,
                                    Eigen::Matrix3d &frame,
                                    Eigen::Vector3d &utility)
        {
            if (!matrix.allFinite())
            {
                return false;
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
                0.5 * (matrix + matrix.transpose()));
            if (solver.info() != Eigen::Success ||
                solver.eigenvalues().maxCoeff() <= kDirectionEpsilon)
            {
                return false;
            }

            for (int i = 0; i < 3; ++i)
            {
                const int source = 2 - i;
                frame.col(i) = solver.eigenvectors().col(source).normalized();
                utility(i) = std::max(solver.eigenvalues()(source), kDirectionEpsilon);
            }
            if (frame.determinant() < 0.0)
            {
                frame.col(2) *= -1.0;
            }
            utility /= utility.maxCoeff();
            return true;
        }

        inline bool pcaDirections(const Eigen::Matrix3Xd &trajSamples,
                                  Eigen::Matrix3d &frame,
                                  Eigen::Vector3d &utility)
        {
            if (trajSamples.cols() < 2)
            {
                return false;
            }
            const Eigen::Vector3d mean = trajSamples.rowwise().mean();
            const Eigen::Matrix3Xd centered = trajSamples.colwise() - mean;
            const Eigen::Matrix3d covariance =
                centered * centered.transpose() /
                static_cast<double>(trajSamples.cols() - 1);
            return eigenDirections(covariance, frame, utility);
        }

        inline bool computeDirections(const Eigen::Matrix3Xd &trajSamples,
                                      const Eigen::Vector3d &tangent,
                                      const Eigen::Vector3d &lateral,
                                      const Parameters &param,
                                      const Eigen::Matrix3d *sensitivityGramian,
                                      Eigen::Matrix3d &frame,
                                      Eigen::Vector3d &utility,
                                      bool &usedFallback)
        {
            usedFallback = false;
            bool success = false;
            if (param.direction_mode == DirectionMode::SENSITIVITY &&
                sensitivityGramian != nullptr)
            {
                success = eigenDirections(*sensitivityGramian, frame, utility);
            }
            else if (param.direction_mode == DirectionMode::PCA)
            {
                success = pcaDirections(trajSamples, frame, utility);
            }
            else if (param.direction_mode == DirectionMode::FRENET)
            {
                success = makeRightHandedFrame(tangent, lateral, frame);
                utility = Eigen::Vector3d(param.tangent_weight,
                                          param.lateral_weight,
                                          param.lateral_weight);
            }

            if (!success)
            {
                usedFallback = true;
                success = pcaDirections(trajSamples, frame, utility);
            }
            if (!success)
            {
                usedFallback = true;
                Eigen::Vector3d primary = tangent;
                if (primary.norm() < kDirectionEpsilon && trajSamples.cols() >= 2)
                {
                    primary = trajSamples.rightCols<1>() - trajSamples.leftCols<1>();
                }
                success = makeRightHandedFrame(primary, lateral, frame);
                utility = Eigen::Vector3d(param.tangent_weight,
                                          param.lateral_weight,
                                          param.lateral_weight);
            }

            if (success && tangent.norm() > kDirectionEpsilon &&
                frame.col(0).dot(tangent) < 0.0)
            {
                frame.col(0) *= -1.0;
                frame.col(2) *= -1.0;
            }
            return success;
        }

        inline Eigen::MatrixX4d boundsToHPoly(const Eigen::Vector3d &anchor,
                                              const Eigen::Matrix3d &frame,
                                              const Eigen::Vector3d &lower,
                                              const Eigen::Vector3d &upper)
        {
            Eigen::MatrixX4d hpoly(6, 4);
            for (int axis = 0; axis < 3; ++axis)
            {
                const Eigen::Vector3d positive = frame.col(axis);
                hpoly.row(2 * axis).head<3>() = positive.transpose();
                hpoly(2 * axis, 3) = -positive.dot(anchor) - upper(axis);

                const Eigen::Vector3d negative = -positive;
                hpoly.row(2 * axis + 1).head<3>() = negative.transpose();
                hpoly(2 * axis + 1, 3) = -negative.dot(anchor) + lower(axis);
            }
            return hpoly;
        }

        inline bool insideBoundary(const Eigen::MatrixX4d &boundary,
                                   const Eigen::Vector3d &anchor,
                                   const Eigen::Matrix3d &frame,
                                   const Eigen::Vector3d &lower,
                                   const Eigen::Vector3d &upper,
                                   const double tolerance = 1.0e-9)
        {
            if (boundary.rows() == 0)
            {
                return true;
            }
            for (int mask = 0; mask < 8; ++mask)
            {
                Eigen::Vector3d local;
                for (int axis = 0; axis < 3; ++axis)
                {
                    local(axis) = (mask & (1 << axis)) ? upper(axis) : lower(axis);
                }
                const Eigen::Vector3d vertex = anchor + frame * local;
                if ((boundary.leftCols<3>() * vertex + boundary.rightCols<1>()).maxCoeff() > tolerance)
                {
                    return false;
                }
            }
            return true;
        }

        inline bool excludesObstacles(const Eigen::MatrixX4d &hpoly,
                                      const Eigen::Matrix3Xd &obstaclePoints,
                                      const double tolerance = 1.0e-9)
        {
            for (int i = 0; i < obstaclePoints.cols(); ++i)
            {
                if ((hpoly.leftCols<3>() * obstaclePoints.col(i) +
                     hpoly.rightCols<1>())
                        .maxCoeff() <= tolerance)
                {
                    return false;
                }
            }
            return true;
        }

        inline double pointSlack(const Eigen::MatrixX4d &hpoly,
                                 const Eigen::Vector3d &point)
        {
            if (hpoly.rows() == 0)
            {
                return -std::numeric_limits<double>::infinity();
            }
            double slack = std::numeric_limits<double>::infinity();
            for (int i = 0; i < hpoly.rows(); ++i)
            {
                const double normalNorm = hpoly.row(i).head<3>().norm();
                if (normalNorm <= kDirectionEpsilon)
                {
                    return -std::numeric_limits<double>::infinity();
                }
                slack = std::min(slack,
                                 -(hpoly.row(i).head<3>().dot(point) + hpoly(i, 3)) /
                                     normalNorm);
            }
            return slack;
        }

    } // namespace detail

    inline bool contains(const Eigen::MatrixX4d &hpoly,
                         const Eigen::Vector3d &point,
                         const double tolerance = 1.0e-9)
    {
        return hpoly.rows() > 0 &&
               (hpoly.leftCols<3>() * point + hpoly.rightCols<1>()).maxCoeff() <= tolerance;
    }

    inline double overlapRadiusAtPoint(const Corridor &lhs,
                                       const Corridor &rhs,
                                       const Eigen::Vector3d &point)
    {
        return std::min(detail::pointSlack(lhs.hpoly, point),
                        detail::pointSlack(rhs.hpoly, point));
    }

    inline bool corridorPenalty(const Corridor &corridor,
                                const Eigen::Vector3d &point,
                                const double epsilon,
                                const double weight,
                                Eigen::Vector3d &gradient,
                                double &cost)
    {
        gradient.setZero();
        cost = 0.0;
        bool active = false;
        for (int i = 0; i < corridor.hpoly.rows(); ++i)
        {
            const Eigen::Vector3d normal = corridor.hpoly.row(i).head<3>().transpose();
            const double violation = normal.dot(point) + corridor.hpoly(i, 3) + epsilon;
            if (violation > 0.0)
            {
                active = true;
                cost += weight * violation * violation;
                gradient.noalias() += 2.0 * weight * violation * normal;
            }
        }
        return active;
    }

    inline bool generateCorridor(const Eigen::MatrixX4d &boundary,
                                 const Eigen::Matrix3Xd &obstaclePoints,
                                 const Eigen::Matrix3Xd &trajSamples,
                                 const Eigen::Vector3d &tangent,
                                 const Eigen::Vector3d &lateral,
                                 Corridor &corridor,
                                 const Parameters &param,
                                 const Eigen::Matrix3d *sensitivityGramian = nullptr)
    {
        const auto started = std::chrono::steady_clock::now();
        corridor = Corridor();
        if (trajSamples.cols() < 2 || param.max_faces < 6 ||
            param.safety_margin < 0.0 || param.inflation_step <= 0.0 ||
            param.max_inflation_distance < 0.0)
        {
            return false;
        }

        Eigen::Matrix3d frame;
        Eigen::Vector3d utility;
        bool usedFallback = false;
        if (!detail::computeDirections(trajSamples, tangent, lateral, param,
                                       sensitivityGramian, frame, utility,
                                       usedFallback))
        {
            return false;
        }

        const Eigen::Vector3d anchor = trajSamples.rowwise().mean();
        const Eigen::Matrix3Xd localSamples =
            (frame.transpose() * trajSamples).colwise() - frame.transpose() * anchor;
        Eigen::Vector3d lower = localSamples.rowwise().minCoeff();
        Eigen::Vector3d upper = localSamples.rowwise().maxCoeff();
        lower.array() -= param.safety_margin;
        upper.array() += param.safety_margin;

        Eigen::MatrixX4d hpoly = detail::boundsToHPoly(anchor, frame, lower, upper);
        if (!detail::insideBoundary(boundary, anchor, frame, lower, upper) ||
            !detail::excludesObstacles(hpoly, obstaclePoints))
        {
            return false;
        }

        std::array<int, 3> axisOrder{{0, 1, 2}};
        std::sort(axisOrder.begin(), axisOrder.end(),
                  [&utility](const int lhs, const int rhs)
                  { return utility(lhs) > utility(rhs); });

        for (const int axis : axisOrder)
        {
            for (int side = 0; side < 2; ++side)
            {
                double expanded = 0.0;
                while (expanded + param.inflation_step <=
                       param.max_inflation_distance + 1.0e-9)
                {
                    Eigen::Vector3d candidateLower = lower;
                    Eigen::Vector3d candidateUpper = upper;
                    if (side == 0)
                    {
                        candidateLower(axis) -= param.inflation_step;
                    }
                    else
                    {
                        candidateUpper(axis) += param.inflation_step;
                    }
                    const Eigen::MatrixX4d candidate =
                        detail::boundsToHPoly(anchor, frame,
                                              candidateLower, candidateUpper);
                    if (!detail::insideBoundary(boundary, anchor, frame,
                                                candidateLower, candidateUpper) ||
                        !detail::excludesObstacles(candidate, obstaclePoints))
                    {
                        break;
                    }
                    lower = candidateLower;
                    upper = candidateUpper;
                    hpoly = candidate;
                    expanded += param.inflation_step;
                }
            }
        }

        corridor.hpoly = hpoly;
        corridor.center = anchor + frame * (0.5 * (lower + upper));
        corridor.frame = frame;
        corridor.utility = utility;
        corridor.face_num = hpoly.rows();
        corridor.weighted_width =
            utility.dot((upper - lower).array().square().matrix());
        corridor.min_sample_slack = std::numeric_limits<double>::infinity();
        for (int i = 0; i < trajSamples.cols(); ++i)
        {
            corridor.min_sample_slack =
                std::min(corridor.min_sample_slack,
                         detail::pointSlack(hpoly, trajSamples.col(i)));
        }
        corridor.direction_fallback = usedFallback;
        corridor.valid = true;
        corridor.generation_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                .count();
        return true;
    }

} // namespace tf_sfc

#endif
