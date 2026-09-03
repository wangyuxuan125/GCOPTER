#ifndef EXACT_SFC_PROJECTION_HPP
#define EXACT_SFC_PROJECTION_HPP

#include "gcopter/minco.hpp"
#include "gcopter/minco_affine_map.hpp"
#include "gcopter/minco_support.hpp"
#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

struct ExactSfcProjectionOptions
{
    // Continuous-time acceptance tolerance.
    double containment_tolerance_m =
        1.0e-6;

    // Numerical tolerance of the finite projection QP.
    double qp_primal_tolerance =
        1.0e-10;

    double qp_dual_tolerance =
        1.0e-12;

    // Numerical safeguards only, not planning hyperparameters.
    int max_exchange_iterations =
        32;

    int max_qp_sweeps =
        20000;
};


struct ExactSfcWitness
{
    bool valid =
        false;

    int piece =
        -1;

    int face =
        -1;

    double normalized_time =
        0.0;

    double physical_time =
        0.0;

    double violation_m =
        -std::numeric_limits<double>::
            infinity();
};


struct ExactTrajectoryCertificate
{
    bool valid =
        false;

    bool contained =
        false;

    int checked_faces =
        0;

    ExactSfcWitness worst;
};


struct ProjectionQpResult
{
    bool success =
        false;

    int sweeps =
        0;

    double max_primal_violation =
        std::numeric_limits<double>::
            infinity();

    double max_dual_change =
        std::numeric_limits<double>::
            infinity();

    Eigen::VectorXd solution;
};


struct ExactSfcProjectionResult
{
    bool success =
        false;

    bool affine_map_valid =
        false;

    bool initial_certificate_valid =
        false;

    bool initial_contained =
        false;

    bool final_certificate_valid =
        false;

    bool final_contained =
        false;

    int exchange_iterations =
        0;

    int active_constraint_count =
        0;

    int total_qp_sweeps =
        0;

    int duplicate_witness_count =
        0;

    double initial_max_violation_m =
        std::numeric_limits<double>::
            infinity();

    double final_max_violation_m =
        std::numeric_limits<double>::
            infinity();

    double correction_l2_m =
        std::numeric_limits<double>::
            infinity();

    double max_waypoint_displacement_m =
        std::numeric_limits<double>::
            infinity();

    double initial_energy =
        std::numeric_limits<double>::
            infinity();

    double final_energy =
        std::numeric_limits<double>::
            infinity();

    double total_ms =
        0.0;

    double certificate_ms =
        0.0;

    double qp_ms =
        0.0;
};


// ================================================================
// Stack Cartesian internal waypoints as
//
//     z = [P_0^T P_1^T ...]^T.
// ================================================================
inline Eigen::VectorXd flattenMincoWaypoints(
    const Eigen::Matrix3Xd &points)
{
    Eigen::VectorXd z(
        3 *
        points.cols());

    for (int waypointId = 0;
         waypointId <
             points.cols();
         ++waypointId)
    {
        z.segment<3>(
            3 *
            waypointId) =
            points.col(
                waypointId);
    }

    return z;
}


inline bool unflattenMincoWaypoints(
    const Eigen::VectorXd &z,
    Eigen::Matrix3Xd &points)
{
    if (z.size() % 3 != 0 ||
        !z.allFinite())
    {
        return false;
    }

    const int waypointCount =
        z.size() / 3;

    points.resize(
        3,
        waypointCount);

    for (int waypointId = 0;
         waypointId <
             waypointCount;
         ++waypointId)
    {
        points.col(
            waypointId) =
            z.segment<3>(
                3 *
                waypointId);
    }

    return points.allFinite();
}


// ================================================================
// Exact continuous-time certificate for a complete one-piece-
// per-polytope trajectory.
// ================================================================
inline ExactTrajectoryCertificate
certifyMincoTrajectoryInCorridors(
    const Trajectory<5> &trajectory,
    const std::vector<Eigen::MatrixX4d>
        &corridors,
    const double containmentToleranceM)
{
    ExactTrajectoryCertificate
        result;

    if (trajectory.getPieceNum() <= 0 ||
        trajectory.getPieceNum() !=
            static_cast<int>(
                corridors.size()))
    {
        return result;
    }

    result.valid =
        true;

    result.contained =
        true;

    result.worst.violation_m =
        -std::numeric_limits<double>::
            infinity();

    for (int pieceId = 0;
         pieceId <
             trajectory.getPieceNum();
         ++pieceId)
    {
        const auto certificate =
            certifyMincoPieceInPolytope(
                trajectory[
                    pieceId],
                corridors[
                    pieceId],
                containmentToleranceM,
                1.0e-10,
                1.0e-12);

        result.checked_faces +=
            certificate
                .checked_face_count;

        if (!certificate.valid)
        {
            result.valid =
                false;

            result.contained =
                false;

            return result;
        }

        if (!certificate.contained)
        {
            result.contained =
                false;
        }

        if (certificate
                .max_signed_violation_m >
            result.worst
                .violation_m)
        {
            result.worst.valid =
                true;

            result.worst.piece =
                pieceId;

            result.worst.face =
                certificate
                    .worst_face;

            result.worst
                .normalized_time =
                    certificate
                        .worst_normalized_time;

            result.worst
                .physical_time =
                    certificate
                        .worst_physical_time;

            result.worst
                .violation_m =
                    certificate
                        .max_signed_violation_m;
        }
    }

    return result;
}


// ================================================================
// Solve
//
//     min_z  0.5 ||z-z0||^2
//     s.t.   a_k^T z <= b_k
//
// for the CURRENT finite active constraint set.
//
// Hildreth's method is dual coordinate ascent. Because the
// Hessian is exactly I, each coordinate update is closed form.
//
// Rows are assumed normalized before entering this function.
// ================================================================
inline ProjectionQpResult
solveEuclideanHalfspaceProjection(
    const Eigen::VectorXd &z0,
    const std::vector<Eigen::VectorXd>
        &rows,
    const std::vector<double> &rhs,
    const double primalTolerance,
    const double dualTolerance,
    const int maxSweeps)
{
    ProjectionQpResult result;

    if (!z0.allFinite() ||
        rows.size() !=
            rhs.size() ||
        maxSweeps <= 0)
    {
        return result;
    }

    if (rows.empty())
    {
        result.success =
            true;

        result.solution =
            z0;

        result.max_primal_violation =
            0.0;

        result.max_dual_change =
            0.0;

        return result;
    }

    Eigen::VectorXd z =
        z0;

    Eigen::VectorXd lambda =
        Eigen::VectorXd::Zero(
            rows.size());

    for (int sweep = 0;
         sweep < maxSweeps;
         ++sweep)
    {
        double maxDualChange =
            0.0;

        for (int constraintId = 0;
             constraintId <
                 static_cast<int>(
                     rows.size());
             ++constraintId)
        {
            const auto &a =
                rows[
                    constraintId];

            if (a.size() !=
                    z0.size() ||
                !a.allFinite() ||
                !std::isfinite(
                    rhs[
                        constraintId]))
            {
                return result;
            }

            // Rows are normalized, so ||a||^2 = 1.
            const double residual =
                a.dot(z) -
                rhs[
                    constraintId];

            const double oldLambda =
                lambda(
                    constraintId);

            const double newLambda =
                std::max(
                    0.0,
                    oldLambda +
                        residual);

            const double delta =
                newLambda -
                oldLambda;

            if (delta != 0.0)
            {
                z -=
                    delta *
                    a;

                lambda(
                    constraintId) =
                    newLambda;
            }

            maxDualChange =
                std::max(
                    maxDualChange,
                    std::abs(
                        delta));
        }

        double maxViolation =
            0.0;

        for (int constraintId = 0;
             constraintId <
                 static_cast<int>(
                     rows.size());
             ++constraintId)
        {
            maxViolation =
                std::max(
                    maxViolation,
                    rows[
                        constraintId]
                            .dot(z) -
                    rhs[
                        constraintId]);
        }

        result.sweeps =
            sweep + 1;

        result.max_primal_violation =
            maxViolation;

        result.max_dual_change =
            maxDualChange;

        if (maxViolation <=
                primalTolerance &&
            maxDualChange <=
                dualTolerance)
        {
            result.success =
                true;

            result.solution =
                z;

            return result;
        }
    }

    result.solution =
        z;

    return result;
}


// ================================================================
// Exact-support exchange method.
//
// Fixed times -> all position SFC constraints are affine in the
// intermediate MINCO waypoints.
//
// At every exchange iteration:
//
//   1. exact continuous-time certification;
//   2. obtain the global worst (piece, face, tau);
//   3. convert it to one exact affine halfspace;
//   4. project P* onto all accumulated hard constraints;
//   5. reconstruct MINCO and certify again.
// ================================================================
inline ExactSfcProjectionResult
projectMincoToExactSfc(
    const Eigen::Matrix3d &headPVA,
    const Eigen::Matrix3d &tailPVA,
    const Eigen::Matrix3Xd &initialPoints,
    const Eigen::VectorXd &times,
    const std::vector<Eigen::MatrixX4d>
        &corridors,
    Trajectory<5> &projectedTrajectory,
    Eigen::Matrix3Xd &projectedPoints,
    const ExactSfcProjectionOptions &options =
        ExactSfcProjectionOptions())
{
    ExactSfcProjectionResult result;

    projectedTrajectory.clear();

    projectedPoints.resize(
        3,
        0);

    const auto totalStarted =
        std::chrono::
            steady_clock::now();

    const int pieceCount =
        times.size();

    if (pieceCount <= 0 ||
        initialPoints.rows() != 3 ||
        initialPoints.cols() !=
            pieceCount - 1 ||
        static_cast<int>(
            corridors.size()) !=
            pieceCount ||
        !headPVA.allFinite() ||
        !tailPVA.allFinite() ||
        !initialPoints.allFinite() ||
        !times.allFinite())
    {
        return result;
    }

    MincoWaypointAffineMap
        affineMap;

    result.affine_map_valid =
        affineMap.build(
            headPVA,
            tailPVA,
            times);

    if (!result.affine_map_valid)
    {
        return result;
    }

    const Eigen::VectorXd z0 =
        flattenMincoWaypoints(
            initialPoints);

    Eigen::VectorXd z =
        z0;

    Eigen::Matrix3Xd currentPoints =
        initialPoints;

    Trajectory<5> currentTrajectory;

    auto buildTrajectory =
        [&](const Eigen::Matrix3Xd &points,
            Trajectory<5> &trajectory,
            double *energy = nullptr)
        {
            minco::MINCO_S3NU m;

            m.setConditions(
                headPVA,
                tailPVA,
                pieceCount);

            m.setParameters(
                points,
                times);

            m.getTrajectory(
                trajectory);

            if (energy != nullptr)
            {
                m.getEnergy(
                    *energy);
            }

            return trajectory
                       .getPieceNum() ==
                       pieceCount;
        };

    if (!buildTrajectory(
            currentPoints,
            currentTrajectory,
            &result.initial_energy))
    {
        return result;
    }

    const auto initialCertStarted =
        std::chrono::
            steady_clock::now();

    ExactTrajectoryCertificate
        certificate =
            certifyMincoTrajectoryInCorridors(
                currentTrajectory,
                corridors,
                options
                    .containment_tolerance_m);

    result.certificate_ms +=
        std::chrono::duration<
            double,
            std::milli>(
                std::chrono::
                    steady_clock::now() -
                initialCertStarted)
            .count();

    result.initial_certificate_valid =
        certificate.valid;

    result.initial_contained =
        certificate.contained;

    result.initial_max_violation_m =
        certificate.worst
            .violation_m;

    if (!certificate.valid)
    {
        return result;
    }

    if (certificate.contained)
    {
        projectedTrajectory =
            currentTrajectory;

        projectedPoints =
            currentPoints;

        result.final_certificate_valid =
            true;

        result.final_contained =
            true;

        result.final_max_violation_m =
            certificate.worst
                .violation_m;

        result.correction_l2_m =
            0.0;

        result.max_waypoint_displacement_m =
            0.0;

        result.final_energy =
            result.initial_energy;

        result.success =
            true;

        result.total_ms =
            std::chrono::duration<
                double,
                std::milli>(
                    std::chrono::
                        steady_clock::now() -
                    totalStarted)
                .count();

        return result;
    }

    std::vector<Eigen::VectorXd>
        activeRows;

    std::vector<double>
        activeRhs;

    for (int exchangeId = 0;
         exchangeId <
             options
                 .max_exchange_iterations;
         ++exchangeId)
    {
        if (!certificate.worst.valid ||
            certificate.worst.piece < 0 ||
            certificate.worst.face < 0)
        {
            return result;
        }

        const int pieceId =
            certificate.worst.piece;

        const int faceId =
            certificate.worst.face;

        const auto &hPoly =
            corridors[
                pieceId];

        if (faceId >=
            hPoly.rows())
        {
            return result;
        }

        const Eigen::Vector3d normal =
            hPoly.block<1, 3>(
                    faceId,
                    0)
                .transpose();

        const double planeOffset =
            hPoly(
                faceId,
                3);

        Eigen::VectorXd row;

        double rhs =
            0.0;

        if (!affineMap
                 .buildHalfspaceConstraintRow(
                     pieceId,
                     certificate.worst
                         .normalized_time,
                     normal,
                     planeOffset,
                     row,
                     rhs))
        {
            return result;
        }

        const double rowNorm =
            row.norm();

        // A zero row means this location is fully determined by
        // fixed boundary conditions. If it is violated, the
        // fixed-time projection problem itself is infeasible.
        if (!std::isfinite(rowNorm) ||
            rowNorm <= 1.0e-12)
        {
            return result;
        }

        row /=
            rowNorm;

        rhs /=
            rowNorm;

        // Numerical duplicate suppression only.
        bool duplicate =
            false;

        for (int constraintId = 0;
             constraintId <
                 static_cast<int>(
                     activeRows.size());
             ++constraintId)
        {
            if ((activeRows[
                     constraintId] -
                 row)
                        .norm() <=
                    1.0e-10 &&
                std::abs(
                    activeRhs[
                        constraintId] -
                    rhs) <=
                    1.0e-10)
            {
                duplicate =
                    true;

                break;
            }
        }

        if (duplicate)
        {
            ++result
                  .duplicate_witness_count;

            // A genuinely violated duplicate after a converged QP
            // indicates numerical inconsistency. Do not hide it by
            // adding a safety margin.
            return result;
        }

        activeRows.push_back(
            row);

        activeRhs.push_back(
            rhs);

        const auto qpStarted =
            std::chrono::
                steady_clock::now();

        const ProjectionQpResult qp =
            solveEuclideanHalfspaceProjection(
                z0,
                activeRows,
                activeRhs,
                options
                    .qp_primal_tolerance,
                options
                    .qp_dual_tolerance,
                options
                    .max_qp_sweeps);

        result.qp_ms +=
            std::chrono::duration<
                double,
                std::milli>(
                    std::chrono::
                        steady_clock::now() -
                    qpStarted)
                .count();

        result.total_qp_sweeps +=
            qp.sweeps;

        if (!qp.success ||
            !qp.solution.allFinite())
        {
            return result;
        }

        z =
            qp.solution;

        if (!unflattenMincoWaypoints(
                z,
                currentPoints))
        {
            return result;
        }

        if (!buildTrajectory(
                currentPoints,
                currentTrajectory))
        {
            return result;
        }

        const auto certStarted =
            std::chrono::
                steady_clock::now();

        certificate =
            certifyMincoTrajectoryInCorridors(
                currentTrajectory,
                corridors,
                options
                    .containment_tolerance_m);

        result.certificate_ms +=
            std::chrono::duration<
                double,
                std::milli>(
                    std::chrono::
                        steady_clock::now() -
                    certStarted)
                .count();

        result.exchange_iterations =
            exchangeId + 1;

        result.active_constraint_count =
            activeRows.size();

        if (!certificate.valid)
        {
            return result;
        }

        if (certificate.contained)
        {
            break;
        }
    }

    projectedTrajectory =
        currentTrajectory;

    projectedPoints =
        currentPoints;

    result.final_certificate_valid =
        certificate.valid;

    result.final_contained =
        certificate.contained;

    result.final_max_violation_m =
        certificate.worst
            .violation_m;

    result.correction_l2_m =
        (z - z0).norm();

    result.max_waypoint_displacement_m =
        0.0;

    for (int waypointId = 0;
         waypointId <
             initialPoints.cols();
         ++waypointId)
    {
        result.max_waypoint_displacement_m =
            std::max(
                result
                    .max_waypoint_displacement_m,
                (currentPoints.col(
                     waypointId) -
                 initialPoints.col(
                     waypointId))
                    .norm());
    }

    double finalEnergy =
        std::numeric_limits<double>::
            infinity();

    Trajectory<5> finalCheckTrajectory;

    if (buildTrajectory(
            currentPoints,
            finalCheckTrajectory,
            &finalEnergy))
    {
        result.final_energy =
            finalEnergy;
    }

    result.success =
        result.final_certificate_valid &&
        result.final_contained;

    result.total_ms =
        std::chrono::duration<
            double,
            std::milli>(
                std::chrono::
                    steady_clock::now() -
                totalStarted)
            .count();

    return result;
}

} // namespace traj_relevant

#endif