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
#include <Eigen/StdVector>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace tf_sfc
{

    enum class DirectionMode
    {
        FRENET = 0,
        PCA = 1,
        SENSITIVITY = 2
    };

    enum class FailureReason
    {
        NONE = 0,
        INVALID_INPUT = 1,
        DIRECTION_FAILURE = 2,
        OUTSIDE_BOUNDARY = 3,
        INITIAL_OBB_OCCUPIED = 4,
        FACE_BUDGET_EXHAUSTED = 5,
        OBSTACLE_SEPARATION_FAILURE = 6
    };

    inline const char *failureReasonName(const FailureReason reason)
    {
        switch (reason)
        {
        case FailureReason::NONE: return "none";
        case FailureReason::INVALID_INPUT: return "invalid_input";
        case FailureReason::DIRECTION_FAILURE: return "direction_failure";
        case FailureReason::OUTSIDE_BOUNDARY: return "outside_boundary";
        case FailureReason::INITIAL_OBB_OCCUPIED: return "initial_obb_occupied";
        case FailureReason::FACE_BUDGET_EXHAUSTED: return "face_budget_exhausted";
        case FailureReason::OBSTACLE_SEPARATION_FAILURE: return "obstacle_separation_failure";
        }
        return "unknown";
    }

    struct Parameters
    {
        int max_faces = 12;
        int max_obs_faces = 6;
        bool enable_obstacle_planes = true;
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
        int obstacle_face_num = 0;
        int obstacle_point_num = 0;
        bool face_budget_saturated = false;
        double generation_time_ms = 0.0;
        double weighted_width = 0.0;
        double min_sample_slack = -std::numeric_limits<double>::infinity();
        double anchor_clearance_radius = -std::numeric_limits<double>::infinity();
        bool valid = false;
        bool direction_fallback = false;
        FailureReason failure_reason = FailureReason::NONE;

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


        inline bool buildFaceBoundedCandidate(
            const Eigen::MatrixX4d &boundary,
            const Eigen::Matrix3Xd &obstaclePoints,
            const Eigen::Matrix3Xd &trajSamples,
            const Eigen::Vector3d &anchor,
            const Eigen::Matrix3d &frame,
            const Eigen::Vector3d &lower,
            const Eigen::Vector3d &upper,
            const Parameters &param,
            Eigen::MatrixX4d &hpoly,
            int &obstacleFaceNum,
            int &obstaclePointNum,
            bool &faceBudgetSaturated,
            FailureReason &failureReason)
        {
            hpoly = boundsToHPoly(anchor, frame, lower, upper);
            obstacleFaceNum = 0;
            obstaclePointNum = 0;
            faceBudgetSaturated = false;
            failureReason = FailureReason::NONE;

            if (!insideBoundary(boundary, anchor, frame, lower, upper))
            {
                failureReason = FailureReason::OUTSIDE_BOUNDARY;
                return false;
            }

            std::vector<int> candidateObstacleIds;
            candidateObstacleIds.reserve(obstaclePoints.cols());
            for (int i = 0; i < obstaclePoints.cols(); ++i)
            {
                if ((hpoly.leftCols<3>() * obstaclePoints.col(i) +
                     hpoly.rightCols<1>()).maxCoeff() <= 1.0e-9)
                {
                    candidateObstacleIds.push_back(i);
                }
            }
            obstaclePointNum = static_cast<int>(candidateObstacleIds.size());
            if (candidateObstacleIds.empty())
            {
                return true;
            }
            if (!param.enable_obstacle_planes)
            {
                failureReason = FailureReason::INITIAL_OBB_OCCUPIED;
                return false;
            }

            const int faceCapacity =
                std::max(0, std::min(param.max_obs_faces,
                                     param.max_faces - 6));
            if (faceCapacity == 0)
            {
                faceBudgetSaturated = true;
                failureReason = FailureReason::FACE_BUDGET_EXHAUSTED;
                return false;
            }

            std::sort(candidateObstacleIds.begin(), candidateObstacleIds.end(),
                      [&obstaclePoints, &trajSamples](const int lhs, const int rhs)
                      {
                          double lhsDistance =
                              std::numeric_limits<double>::infinity();
                          double rhsDistance =
                              std::numeric_limits<double>::infinity();
                          for (int j = 0; j < trajSamples.cols(); ++j)
                          {
                              lhsDistance = std::min(
                                  lhsDistance,
                                  (obstaclePoints.col(lhs) -
                                   trajSamples.col(j)).squaredNorm());
                              rhsDistance = std::min(
                                  rhsDistance,
                                  (obstaclePoints.col(rhs) -
                                   trajSamples.col(j)).squaredNorm());
                          }
                          return lhsDistance < rhsDistance;
                      });

            std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> cutNormals;
            std::vector<double> cutOffsets;
            const double requiredSampleSlack =
                std::max(param.min_overlap_radius,
                         0.5 * param.safety_margin);
            constexpr double kNormalMergeCosine = 0.985;
            constexpr double kTolerance = 1.0e-9;

            for (const int obstacleId : candidateObstacleIds)
            {
                const Eigen::Vector3d obstacle =
                    obstaclePoints.col(obstacleId);
                bool alreadyExcluded = false;
                for (size_t faceId = 0; faceId < cutNormals.size(); ++faceId)
                {
                    if (cutNormals[faceId].dot(obstacle) >
                        cutOffsets[faceId] + kTolerance)
                    {
                        alreadyExcluded = true;
                        break;
                    }
                }
                if (alreadyExcluded)
                {
                    continue;
                }

                int nearestSampleId = -1;
                double nearestDistance =
                    std::numeric_limits<double>::infinity();
                for (int sampleId = 0; sampleId < trajSamples.cols();
                     ++sampleId)
                {
                    const double distance =
                        (obstacle - trajSamples.col(sampleId)).squaredNorm();
                    if (distance < nearestDistance)
                    {
                        nearestDistance = distance;
                        nearestSampleId = sampleId;
                    }
                }
                if (nearestSampleId < 0 || nearestDistance <= 1.0e-12)
                {
                    failureReason =
                        FailureReason::OBSTACLE_SEPARATION_FAILURE;
                    return false;
                }

                const Eigen::Vector3d normal =
                    (obstacle - trajSamples.col(nearestSampleId)).normalized();
                const double offset =
                    normal.dot(obstacle) - param.safety_margin;
                double sampleSupport =
                    -std::numeric_limits<double>::infinity();
                for (int sampleId = 0; sampleId < trajSamples.cols();
                     ++sampleId)
                {
                    sampleSupport = std::max(
                        sampleSupport,
                        normal.dot(trajSamples.col(sampleId)));
                }
                if (sampleSupport + requiredSampleSlack >
                    offset + kTolerance)
                {
                    failureReason =
                        FailureReason::OBSTACLE_SEPARATION_FAILURE;
                    return false;
                }

                int mergeId = -1;
                for (size_t faceId = 0; faceId < cutNormals.size(); ++faceId)
                {
                    if (cutNormals[faceId].dot(normal) >=
                        kNormalMergeCosine)
                    {
                        mergeId = static_cast<int>(faceId);
                        break;
                    }
                }
                if (mergeId >= 0)
                {
                    const double mergedOffset =
                        std::min(cutOffsets[mergeId], offset);
                    double mergedSupport =
                        -std::numeric_limits<double>::infinity();
                    for (int sampleId = 0;
                         sampleId < trajSamples.cols(); ++sampleId)
                    {
                        mergedSupport = std::max(
                            mergedSupport,
                            cutNormals[mergeId].dot(
                                trajSamples.col(sampleId)));
                    }
                    if (mergedSupport + requiredSampleSlack >
                        mergedOffset + kTolerance)
                    {
                        failureReason =
                            FailureReason::OBSTACLE_SEPARATION_FAILURE;
                        return false;
                    }
                    cutOffsets[mergeId] = mergedOffset;
                }
                else
                {
                    if (static_cast<int>(cutNormals.size()) >=
                        faceCapacity)
                    {
                        faceBudgetSaturated = true;
                        failureReason =
                            FailureReason::FACE_BUDGET_EXHAUSTED;
                        return false;
                    }
                    cutNormals.push_back(normal);
                    cutOffsets.push_back(offset);
                }
            }

            hpoly.conservativeResize(
                6 + static_cast<int>(cutNormals.size()), 4);
            for (size_t faceId = 0; faceId < cutNormals.size(); ++faceId)
            {
                hpoly.row(6 + static_cast<int>(faceId)).head<3>() =
                    cutNormals[faceId].transpose();
                hpoly(6 + static_cast<int>(faceId), 3) =
                    -cutOffsets[faceId];
            }

            for (int sampleId = 0; sampleId < trajSamples.cols();
                 ++sampleId)
            {
                if ((hpoly.leftCols<3>() * trajSamples.col(sampleId) +
                     hpoly.rightCols<1>()).maxCoeff() > kTolerance)
                {
                    failureReason =
                        FailureReason::OBSTACLE_SEPARATION_FAILURE;
                    return false;
                }
            }
            for (const int obstacleId : candidateObstacleIds)
            {
                if ((hpoly.leftCols<3>() *
                         obstaclePoints.col(obstacleId) +
                     hpoly.rightCols<1>()).maxCoeff() <= kTolerance)
                {
                    faceBudgetSaturated =
                        static_cast<int>(cutNormals.size()) >=
                        faceCapacity;
                    failureReason = faceBudgetSaturated
                                        ? FailureReason::FACE_BUDGET_EXHAUSTED
                                        : FailureReason::OBSTACLE_SEPARATION_FAILURE;
                    return false;
                }
            }

            obstacleFaceNum = static_cast<int>(cutNormals.size());
            faceBudgetSaturated = obstacleFaceNum >= faceCapacity;
            return true;
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
            param.max_obs_faces < 0 || param.safety_margin < 0.0 ||
            param.inflation_step <= 0.0 ||
            param.max_inflation_distance < 0.0)
        {
            corridor.failure_reason = FailureReason::INVALID_INPUT;
            return false;
        }

        Eigen::Matrix3d frame;
        Eigen::Vector3d utility;
        bool usedFallback = false;
        if (!detail::computeDirections(trajSamples, tangent, lateral, param,
                                       sensitivityGramian, frame, utility,
                                       usedFallback))
        {
            corridor.failure_reason = FailureReason::DIRECTION_FAILURE;
            return false;
        }

        const Eigen::Vector3d anchor = trajSamples.rowwise().mean();
        const Eigen::Matrix3Xd localSamples =
            (frame.transpose() * trajSamples).colwise() -
            frame.transpose() * anchor;
        Eigen::Vector3d lower = localSamples.rowwise().minCoeff();
        Eigen::Vector3d upper = localSamples.rowwise().maxCoeff();
        lower.array() -= param.safety_margin;
        upper.array() += param.safety_margin;

        Eigen::MatrixX4d acceptedHpoly;
        int acceptedObstacleFaces = 0;
        int acceptedObstaclePoints = 0;
        bool acceptedBudgetSaturated = false;
        FailureReason candidateFailure = FailureReason::NONE;
        if (!detail::buildFaceBoundedCandidate(
                boundary, obstaclePoints, trajSamples, anchor, frame,
                lower, upper, param, acceptedHpoly,
                acceptedObstacleFaces, acceptedObstaclePoints,
                acceptedBudgetSaturated, candidateFailure))
        {
            corridor.failure_reason = candidateFailure;
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

                    Eigen::MatrixX4d candidateHpoly;
                    int candidateObstacleFaces = 0;
                    int candidateObstaclePoints = 0;
                    bool candidateBudgetSaturated = false;
                    FailureReason expansionFailure =
                        FailureReason::NONE;
                    if (!detail::buildFaceBoundedCandidate(
                            boundary, obstaclePoints, trajSamples,
                            anchor, frame, candidateLower,
                            candidateUpper, param, candidateHpoly,
                            candidateObstacleFaces,
                            candidateObstaclePoints,
                            candidateBudgetSaturated,
                            expansionFailure))
                    {
                        break;
                    }

                    lower = candidateLower;
                    upper = candidateUpper;
                    acceptedHpoly = candidateHpoly;
                    acceptedObstacleFaces = candidateObstacleFaces;
                    acceptedObstaclePoints = candidateObstaclePoints;
                    acceptedBudgetSaturated =
                        candidateBudgetSaturated;
                    expanded += param.inflation_step;
                }
            }
        }

        corridor.hpoly = acceptedHpoly;
        corridor.center = anchor + frame * (0.5 * (lower + upper));
        corridor.frame = frame;
        corridor.utility = utility;
        corridor.face_num = acceptedHpoly.rows();
        corridor.obstacle_face_num = acceptedObstacleFaces;
        corridor.obstacle_point_num = acceptedObstaclePoints;
        corridor.face_budget_saturated = acceptedBudgetSaturated;
        corridor.weighted_width =
            utility.dot((upper - lower).array().square().matrix());
        corridor.min_sample_slack =
            std::numeric_limits<double>::infinity();
        for (int i = 0; i < trajSamples.cols(); ++i)
        {
            corridor.min_sample_slack =
                std::min(corridor.min_sample_slack,
                         detail::pointSlack(acceptedHpoly,
                                            trajSamples.col(i)));
        }
        corridor.anchor_clearance_radius =
            detail::pointSlack(acceptedHpoly, anchor);
        corridor.direction_fallback = usedFallback;
        corridor.valid = true;
        corridor.failure_reason = FailureReason::NONE;
        corridor.generation_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                .count();
        return true;
    }

} // namespace tf_sfc

#endif
