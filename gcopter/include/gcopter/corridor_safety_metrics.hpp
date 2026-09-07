#pragma once

#include "gcopter/sdlp.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>


namespace gcopter_benchmark
{

struct CommonCorridorSafetyMetric
{
    bool valid = false;

    // --------------------------------------------------------
    // Common dilated-surface obstacle certificate.
    // --------------------------------------------------------
    bool obstacle_surface_safe = false;

    int obstacle_sample_count = 0;

    int worst_obstacle_index = -1;

    // For obstacle point o:
    //
    //     m(o) =
    //         max_j
    //         (n_j^T o + d_j) / ||n_j||
    //
    // m > 0:
    //     excluded by at least one halfspace.
    //
    // m = 0:
    //     touches corridor boundary.
    //
    // m < 0:
    //     lies inside every halfspace -> penetrates corridor.
    //
    // We report:
    //
    //     min_o m(o).
    double min_obstacle_exclusion_margin_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_obstacle_penetration_m =
        std::numeric_limits<double>::
            quiet_NaN();


    // --------------------------------------------------------
    // Independent finite-map containment.
    // --------------------------------------------------------
    bool map_contained = false;

    // Positive:
    //     corridor exceeds map bounds.
    //
    // Zero:
    //     touches map bounds.
    //
    // Negative:
    //     strictly inside.
    double max_map_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();


    // Final common certificate:
    //
    //     obstacle_surface_safe &&
    //     map_contained
    bool safe = false;
};


// ============================================================
// Common independent corridor safety verifier.
//
// H convention:
//
//     n^T x + d <= 0.
//
// Part A:
//     test EVERY point in the common global dilated obstacle
//     surface cloud.
//
// Part B:
//     independently solve six support LPs to verify that the
//     full H-polytope lies inside the finite map bounds.
//
// IMPORTANT:
//
// Boundary contact with a dilated obstacle surface sample is
// accepted as non-penetration:
//
//     min exclusion margin >= -tolerance.
//
// This matches tangent-plane SFC semantics.  Positive clearance
// is still reported explicitly through the signed margin.
//
// This function performs MEASUREMENT ONLY.
// It never modifies the H-polytope.
// ============================================================
inline CommonCorridorSafetyMetric
evaluateCommonCorridorSafety(
    const Eigen::MatrixX4d &hPoly,
    const std::vector<Eigen::Vector3d> &obstacleSurface,
    const Eigen::Vector3d &mapLow,
    const Eigen::Vector3d &mapHigh,
    const double safetyToleranceM =
        1.0e-8,
    const double normalEpsilon =
        1.0e-12)
{
    CommonCorridorSafetyMetric result;

    result.obstacle_sample_count =
        static_cast<int>(
            obstacleSurface.size());


    if (hPoly.rows() < 4 ||
        !hPoly.allFinite() ||
        !mapLow.allFinite() ||
        !mapHigh.allFinite() ||
        (mapHigh.array() <=
         mapLow.array()).any() ||
        !std::isfinite(
            safetyToleranceM) ||
        safetyToleranceM < 0.0 ||
        !std::isfinite(
            normalEpsilon) ||
        normalEpsilon <= 0.0)
    {
        return result;
    }


    const int faceCount =
        static_cast<int>(
            hPoly.rows());


    // --------------------------------------------------------
    // Normalize all H rows.
    //
    // After normalization, every H-plane residual is a metric
    // signed distance in metres along that plane normal.
    // --------------------------------------------------------
    Eigen::MatrixX4d normalizedH(
        faceCount,
        4);


    for (int faceId = 0;
         faceId < faceCount;
         ++faceId)
    {
        const Eigen::Vector3d normal =
            hPoly.row(faceId)
                .head<3>()
                .transpose();

        const double normalNorm =
            normal.norm();


        if (!std::isfinite(normalNorm) ||
            normalNorm <= normalEpsilon)
        {
            return result;
        }


        normalizedH.row(faceId) =
            hPoly.row(faceId) /
            normalNorm;
    }


    // ========================================================
    // A. Global dilated-surface exclusion.
    // ========================================================
    if (obstacleSurface.empty())
    {
        // Empty obstacle environment is vacuously safe.
        result.obstacle_surface_safe =
            true;

        result.max_obstacle_penetration_m =
            0.0;
    }
    else
    {
        double minimumExclusionMarginM =
            std::numeric_limits<double>::
                infinity();

        int worstObstacleIndex =
            -1;


        for (int obstacleId = 0;
             obstacleId <
                 static_cast<int>(
                     obstacleSurface.size());
             ++obstacleId)
        {
            const Eigen::Vector3d &point =
                obstacleSurface[
                    obstacleId];


            if (!point.allFinite())
            {
                return result;
            }


            const Eigen::Vector4d pointH(
                point.x(),
                point.y(),
                point.z(),
                1.0);


            // A point is outside the convex polytope iff at
            // least one H-plane has positive residual.
            const double exclusionMarginM =
                (
                    normalizedH *
                    pointH)
                    .maxCoeff();


            if (!std::isfinite(
                    exclusionMarginM))
            {
                return result;
            }


            if (exclusionMarginM <
                minimumExclusionMarginM)
            {
                minimumExclusionMarginM =
                    exclusionMarginM;

                worstObstacleIndex =
                    obstacleId;
            }
        }


        result.min_obstacle_exclusion_margin_m =
            minimumExclusionMarginM;

        result.worst_obstacle_index =
            worstObstacleIndex;

        result.max_obstacle_penetration_m =
            std::max(
                0.0,
                -minimumExclusionMarginM);

        result.obstacle_surface_safe =
            minimumExclusionMarginM >=
                -safetyToleranceM;
    }


    // ========================================================
    // B. Independent finite-map containment.
    //
    // Solve six LPs:
    //
    //     min / max x
    //     min / max y
    //     min / max z
    //
    // subject to the complete H-polytope.
    // ========================================================
    Eigen::Matrix<double,
                  Eigen::Dynamic,
                  3>
        A(
            faceCount,
            3);

    Eigen::VectorXd b(
        faceCount);


    A =
        normalizedH
            .leftCols<3>();

    b =
        -normalizedH
             .rightCols<1>();


    double maxMapViolationM =
        -std::numeric_limits<double>::
            infinity();


    for (int axis = 0;
         axis < 3;
         ++axis)
    {
        Eigen::Vector3d positiveObjective =
            Eigen::Vector3d::Zero();

        positiveObjective(axis) =
            1.0;


        Eigen::Vector3d negativeObjective =
            -positiveObjective;


        Eigen::Vector3d minPoint =
            Eigen::Vector3d::Zero();

        Eigen::Vector3d maxPoint =
            Eigen::Vector3d::Zero();


        // min x_axis
        const double minimumCoordinate =
            sdlp::linprog<3>(
                positiveObjective,
                A,
                b,
                minPoint);


        // min (-x_axis) = -max(x_axis)
        const double negativeMaximumCoordinate =
            sdlp::linprog<3>(
                negativeObjective,
                A,
                b,
                maxPoint);


        // Every benchmark corridor must be feasible and
        // bounded.  +/- infinity therefore means the safety
        // measurement itself is invalid.
        if (!std::isfinite(
                minimumCoordinate) ||
            !std::isfinite(
                negativeMaximumCoordinate))
        {
            return result;
        }


        const double maximumCoordinate =
            -negativeMaximumCoordinate;


        const double upperViolationM =
            maximumCoordinate -
            mapHigh(axis);

        const double lowerViolationM =
            mapLow(axis) -
            minimumCoordinate;


        maxMapViolationM =
            std::max(
                maxMapViolationM,
                std::max(
                    upperViolationM,
                    lowerViolationM));
    }


    if (!std::isfinite(
            maxMapViolationM))
    {
        return result;
    }


    result.max_map_violation_m =
        maxMapViolationM;

    result.map_contained =
        maxMapViolationM <=
            safetyToleranceM;


    result.safe =
        result.obstacle_surface_safe &&
        result.map_contained;


    result.valid =
        true;


    return result;
}

} // namespace gcopter_benchmark