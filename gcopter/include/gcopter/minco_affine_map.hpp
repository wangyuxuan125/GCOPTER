#ifndef MINCO_AFFINE_MAP_HPP
#define MINCO_AFFINE_MAP_HPP

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

// ================================================================
// Fixed-time affine representation of a quintic MINCO trajectory.
//
// For fixed
//
//     head PVA,
//     tail PVA,
//     piece times T,
//
// MINCO is linear in the intermediate Cartesian waypoints:
//
//     p_i(tau; P)
//       = c_i(tau)
//       + sum_j beta_{ij}(tau) P_j.
//
// Because x/y/z are solved by the same scalar banded system,
// beta_{ij} is a SCALAR shared by all three coordinates.
//
// This representation contains no finite differences.
// The basis responses are obtained by exact linear superposition
// through MINCO itself, up to floating-point roundoff.
// ================================================================
class MincoWaypointAffineMap
{
private:
    bool valid_ =
        false;

    int piece_count_ =
        0;

    int waypoint_count_ =
        0;

    Eigen::VectorXd times_;

    Trajectory<5>
        offset_trajectory_;

    std::vector<Trajectory<5>>
        basis_trajectories_;

public:
    inline bool build(
        const Eigen::Matrix3d &headPVA,
        const Eigen::Matrix3d &tailPVA,
        const Eigen::VectorXd &times)
    {
        valid_ =
            false;

        piece_count_ =
            static_cast<int>(
                times.size());

        waypoint_count_ =
            std::max(
                piece_count_ - 1,
                0);

        times_ =
            times;

        offset_trajectory_.clear();

        basis_trajectories_.clear();

        if (piece_count_ <= 0 ||
            !headPVA.allFinite() ||
            !tailPVA.allFinite() ||
            !times.allFinite())
        {
            return false;
        }

        for (int pieceId = 0;
             pieceId < piece_count_;
             ++pieceId)
        {
            if (!std::isfinite(
                    times(pieceId)) ||
                times(pieceId) <=
                    0.0)
            {
                return false;
            }
        }

        // --------------------------------------------------------
        // Affine offset:
        //
        //     c(tau) = p(tau; P = 0)
        //
        // with the REAL fixed head/tail boundary conditions.
        // --------------------------------------------------------
        Eigen::Matrix3Xd zeroPoints =
            Eigen::Matrix3Xd::Zero(
                3,
                waypoint_count_);

        minco::MINCO_S3NU
            offsetMinco;

        offsetMinco.setConditions(
            headPVA,
            tailPVA,
            piece_count_);

        offsetMinco.setParameters(
            zeroPoints,
            times_);

        offsetMinco.getTrajectory(
            offset_trajectory_);

        if (offset_trajectory_
                .getPieceNum() !=
            piece_count_)
        {
            return false;
        }

        // --------------------------------------------------------
        // Linear basis:
        //
        // Zero boundary PVA eliminates the affine offset.
        //
        // For waypoint j set only its x coordinate to one.
        // Since all coordinates share the same scalar MINCO
        // system, the resulting x response is beta_j(tau),
        // applicable equally to x/y/z.
        // --------------------------------------------------------
        const Eigen::Matrix3d
            zeroBoundary =
                Eigen::Matrix3d::Zero();

        minco::MINCO_S3NU
            basisMinco;

        basisMinco.setConditions(
            zeroBoundary,
            zeroBoundary,
            piece_count_);

        basis_trajectories_.reserve(
            waypoint_count_);

        for (int waypointId = 0;
             waypointId <
                 waypoint_count_;
             ++waypointId)
        {
            Eigen::Matrix3Xd
                basisPoints =
                    Eigen::Matrix3Xd::Zero(
                        3,
                        waypoint_count_);

            basisPoints(
                0,
                waypointId) =
                1.0;

            basisMinco.setParameters(
                basisPoints,
                times_);

            Trajectory<5>
                basisTrajectory;

            basisMinco.getTrajectory(
                basisTrajectory);

            if (basisTrajectory
                    .getPieceNum() !=
                piece_count_)
            {
                basis_trajectories_.clear();

                return false;
            }

            basis_trajectories_
                .push_back(
                    basisTrajectory);
        }

        // --------------------------------------------------------
        // Numerical validity check.
        // --------------------------------------------------------
        for (int pieceId = 0;
             pieceId < piece_count_;
             ++pieceId)
        {
            const auto &offsetPiece =
                offset_trajectory_[
                    pieceId];

            if (!std::isfinite(
                    offsetPiece
                        .getDuration()) ||
                offsetPiece
                        .getDuration() <=
                    0.0 ||
                !offsetPiece
                     .getCoeffMat()
                     .allFinite())
            {
                return false;
            }

            for (int waypointId = 0;
                 waypointId <
                     waypoint_count_;
                 ++waypointId)
            {
                const auto &basisPiece =
                    basis_trajectories_[
                        waypointId][
                        pieceId];

                if (!basisPiece
                         .getCoeffMat()
                         .allFinite())
                {
                    return false;
                }
            }
        }

        valid_ =
            true;

        return true;
    }


    inline bool valid() const
    {
        return valid_;
    }


    inline int pieceCount() const
    {
        return piece_count_;
    }


    inline int waypointCount() const
    {
        return waypoint_count_;
    }


    inline int variableDimension() const
    {
        return 3 *
            waypoint_count_;
    }


    // ------------------------------------------------------------
    // Return
    //
    //     p_i(tau; P)
    //       = offset
    //       + sum_j beta(j) P_j
    //
    // tau is normalized time in [0,1].
    // ------------------------------------------------------------
    inline bool positionAffineCoefficients(
        const int pieceId,
        const double normalizedTime,
        Eigen::Vector3d &offset,
        Eigen::VectorXd &beta) const
    {
        offset.setZero();

        beta.resize(0);

        if (!valid_ ||
            pieceId < 0 ||
            pieceId >=
                piece_count_ ||
            !std::isfinite(
                normalizedTime))
        {
            return false;
        }

        const double tau =
            std::max(
                0.0,
                std::min(
                    1.0,
                    normalizedTime));

        const double localTime =
            tau *
            times_(pieceId);

        offset =
            offset_trajectory_[
                pieceId]
                .getPos(
                    localTime);

        if (!offset.allFinite())
        {
            return false;
        }

        beta.resize(
            waypoint_count_);

        for (int waypointId = 0;
             waypointId <
                 waypoint_count_;
             ++waypointId)
        {
            const Eigen::Vector3d
                basisPosition =
                    basis_trajectories_[
                        waypointId][
                        pieceId]
                        .getPos(
                            localTime);

            if (!basisPosition
                     .allFinite())
            {
                return false;
            }

            beta(
                waypointId) =
                basisPosition.x();
        }

        return beta.allFinite();
    }


    inline bool evaluatePosition(
        const int pieceId,
        const double normalizedTime,
        const Eigen::Matrix3Xd &innerPoints,
        Eigen::Vector3d &position) const
    {
        position.setZero();

        if (!valid_ ||
            innerPoints.rows() != 3 ||
            innerPoints.cols() !=
                waypoint_count_ ||
            !innerPoints.allFinite())
        {
            return false;
        }

        Eigen::Vector3d offset;

        Eigen::VectorXd beta;

        if (!positionAffineCoefficients(
                pieceId,
                normalizedTime,
                offset,
                beta))
        {
            return false;
        }

        position =
            offset;

        for (int waypointId = 0;
             waypointId <
                 waypoint_count_;
             ++waypointId)
        {
            position +=
                beta(
                    waypointId) *
                innerPoints.col(
                    waypointId);
        }

        return position.allFinite();
    }


    // ------------------------------------------------------------
    // Build one HARD linear halfspace row.
    //
    // Original corridor face:
    //
    //     n^T p_i(tau; P) + d <= 0.
    //
    // Stack variables column-wise:
    //
    //     z = [P_0^T, P_1^T, ...]^T.
    //
    // Then:
    //
    //     row^T z <= rhs.
    // ------------------------------------------------------------
    inline bool buildHalfspaceConstraintRow(
        const int pieceId,
        const double normalizedTime,
        const Eigen::Vector3d &normal,
        const double planeOffset,
        Eigen::VectorXd &row,
        double &rhs) const
    {
        row.resize(0);

        rhs =
            0.0;

        if (!normal.allFinite() ||
            !std::isfinite(
                planeOffset))
        {
            return false;
        }

        Eigen::Vector3d offset;

        Eigen::VectorXd beta;

        if (!positionAffineCoefficients(
                pieceId,
                normalizedTime,
                offset,
                beta))
        {
            return false;
        }

        row =
            Eigen::VectorXd::Zero(
                variableDimension());

        for (int waypointId = 0;
             waypointId <
                 waypoint_count_;
             ++waypointId)
        {
            row.segment<3>(
                3 *
                waypointId) =
                beta(
                    waypointId) *
                normal;
        }

        rhs =
            -planeOffset -
            normal.dot(
                offset);

        return row.allFinite() &&
               std::isfinite(
                   rhs);
    }
};


struct MincoAffineValidationResult
{
    bool valid =
        false;

    int test_trajectory_count =
        0;

    int position_test_count =
        0;

    double max_position_error_m =
        std::numeric_limits<double>::
            infinity();

    double max_relative_error =
        std::numeric_limits<double>::
            infinity();

    double validation_ms =
        0.0;
};


// ================================================================
// Deterministic numerical validation.
//
// We compare:
//
//     direct MINCO(testPoints)
//
// against
//
//     affineMap(testPoints)
//
// for several deterministic waypoint perturbations and many
// normalized times.
//
// No random generator and no finite-difference derivative are used.
// ================================================================
inline MincoAffineValidationResult
validateMincoWaypointAffineMap(
    const MincoWaypointAffineMap &affineMap,
    const Eigen::Matrix3d &headPVA,
    const Eigen::Matrix3d &tailPVA,
    const Eigen::VectorXd &times,
    const Eigen::Matrix3Xd &referencePoints,
    const int testTrajectoryCount = 16,
    const int samplesPerPiece = 12,
    const double perturbationAmplitude = 0.5)
{
    MincoAffineValidationResult
        result;

    const auto started =
        std::chrono::steady_clock::now();

    if (!affineMap.valid() ||
        affineMap.pieceCount() !=
            times.size() ||
        referencePoints.rows() != 3 ||
        referencePoints.cols() !=
            affineMap.waypointCount() ||
        !referencePoints.allFinite() ||
        testTrajectoryCount <= 0 ||
        samplesPerPiece <= 0 ||
        !std::isfinite(
            perturbationAmplitude) ||
        perturbationAmplitude < 0.0)
    {
        return result;
    }

    result.max_position_error_m =
        0.0;

    result.max_relative_error =
        0.0;

    for (int testId = 0;
         testId <
             testTrajectoryCount;
         ++testId)
    {
        Eigen::Matrix3Xd
            testPoints =
                referencePoints;

        // --------------------------------------------------------
        // Deterministic nontrivial perturbation.
        // --------------------------------------------------------
        for (int waypointId = 0;
             waypointId <
                 testPoints.cols();
             ++waypointId)
        {
            for (int axis = 0;
                 axis < 3;
                 ++axis)
            {
                const double phase =
                    0.731 *
                    static_cast<double>(
                        testId + 1) *
                    static_cast<double>(
                        3 *
                            waypointId +
                        axis +
                        1);

                testPoints(
                    axis,
                    waypointId) +=
                    perturbationAmplitude *
                    std::sin(
                        phase);
            }
        }

        minco::MINCO_S3NU
            directMinco;

        directMinco.setConditions(
            headPVA,
            tailPVA,
            affineMap.pieceCount());

        directMinco.setParameters(
            testPoints,
            times);

        Trajectory<5>
            directTrajectory;

        directMinco.getTrajectory(
            directTrajectory);

        if (directTrajectory
                .getPieceNum() !=
            affineMap.pieceCount())
        {
            return result;
        }

        ++result
              .test_trajectory_count;

        for (int pieceId = 0;
             pieceId <
                 affineMap
                     .pieceCount();
             ++pieceId)
        {
            const double duration =
                directTrajectory[
                    pieceId]
                    .getDuration();

            for (int sampleId = 0;
                 sampleId <=
                     samplesPerPiece;
                 ++sampleId)
            {
                const double tau =
                    static_cast<double>(
                        sampleId) /
                    static_cast<double>(
                        samplesPerPiece);

                const Eigen::Vector3d
                    directPosition =
                        directTrajectory[
                            pieceId]
                            .getPos(
                                tau *
                                duration);

                Eigen::Vector3d
                    affinePosition;

                if (!affineMap
                         .evaluatePosition(
                             pieceId,
                             tau,
                             testPoints,
                             affinePosition))
                {
                    return result;
                }

                const double error =
                    (directPosition -
                     affinePosition)
                        .norm();

                const double scale =
                    std::max(
                        directPosition
                            .norm(),
                        1.0);

                const double relativeError =
                    error /
                    scale;

                result.max_position_error_m =
                    std::max(
                        result
                            .max_position_error_m,
                        error);

                result.max_relative_error =
                    std::max(
                        result
                            .max_relative_error,
                        relativeError);

                ++result
                      .position_test_count;
            }
        }
    }

    result.validation_ms =
        std::chrono::duration<
            double,
            std::milli>(
                std::chrono::
                    steady_clock::now() -
                started)
            .count();

    // 1 nm is deliberately much tighter than any corridor
    // tolerance we will use later.
    result.valid =
        std::isfinite(
            result.max_position_error_m) &&
        std::isfinite(
            result.max_relative_error) &&
        result.max_position_error_m <=
            1.0e-9;

    return result;
}

} // namespace traj_relevant

#endif