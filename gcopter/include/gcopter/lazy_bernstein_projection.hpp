#ifndef LAZY_BERNSTEIN_PROJECTION_HPP
#define LAZY_BERNSTEIN_PROJECTION_HPP

#include "gcopter/bernstein_sfc_projection.hpp"
#include "gcopter/exact_sfc_projection.hpp"
#include "gcopter/active_set_projection.hpp"
#include "gcopter/root_finder.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <set>

namespace traj_relevant
{

struct ExactFaceViolationInterval
{
    bool valid = false;

    double begin = 0.0;
    double end = 1.0;

    int interior_root_count = 0;
};


template <int D>
inline ExactFaceViolationInterval
exactFaceViolationInterval(
    const Piece<D> &piece,
    const Eigen::Vector3d &normal,
    const double planeOffset,
    const double witnessTau,
    const double rootTolerance = 1.0e-10,
    const double coefficientTolerance = 1.0e-12)
{
    ExactFaceViolationInterval result;

    if (!normal.allFinite() ||
        !std::isfinite(planeOffset) ||
        !std::isfinite(witnessTau) ||
        witnessTau < 0.0 ||
        witnessTau > 1.0)
    {
        return result;
    }

    const double normalNorm =
        normal.norm();

    if (!std::isfinite(normalNorm) ||
        normalNorm <= coefficientTolerance)
    {
        return result;
    }

    const auto coeffMat =
        piece.normalizePosCoeffMat();

    Eigen::VectorXd coeffs(
        D + 1);

    for (int coefficientId = 0;
         coefficientId <= D;
         ++coefficientId)
    {
        coeffs(coefficientId) =
            normal.dot(
                coeffMat.col(
                    coefficientId));
    }

    // Storage is descending:
    //
    //   coeffs(0) = tau^D
    //   ...
    //   coeffs(D) = constant.
    coeffs(D) +=
        planeOffset;

    if (!coeffs.allFinite())
    {
        return result;
    }

    const double scale =
        coeffs.cwiseAbs()
            .maxCoeff();

    if (!std::isfinite(scale) ||
        scale <= coefficientTolerance)
    {
        return result;
    }

    const double witnessValue =
        RootFinder::polyVal(
            coeffs,
            witnessTau,
            true);

    // This helper is intended only for a genuinely
    // violated face witness.
    if (!std::isfinite(witnessValue) ||
        witnessValue <= 0.0)
    {
        return result;
    }

    const Eigen::VectorXd scaled =
        coeffs / scale;

    // RootFinder's isolation routine assumes nonzero
    // polynomial values at the search boundaries.
    //
    // Boundary roots themselves do not need to be returned:
    // begin/end already default to 0/1.
    double searchBegin =
        0.0;

    double searchEnd =
        1.0;

    if (std::abs(
            RootFinder::polyVal(
                scaled,
                0.0,
                true)) <=
        coefficientTolerance)
    {
        searchBegin =
            rootTolerance;
    }

    if (std::abs(
            RootFinder::polyVal(
                scaled,
                1.0,
                true)) <=
        coefficientTolerance)
    {
        searchEnd =
            1.0 -
            rootTolerance;
    }

    std::set<double> roots;

    if (searchBegin <
        searchEnd)
    {
        roots =
            RootFinder::solvePolynomial(
                scaled,
                searchBegin,
                searchEnd,
                rootTolerance);
    }

    result.interior_root_count =
        static_cast<int>(
            roots.size());

    double leftRoot =
        0.0;

    double rightRoot =
        1.0;

    for (const double root : roots)
    {
        if (!std::isfinite(root))
        {
            continue;
        }

        if (root <
            witnessTau -
                rootTolerance)
        {
            leftRoot =
                std::max(
                    leftRoot,
                    root);
        }
        else if (root >
                 witnessTau +
                     rootTolerance)
        {
            rightRoot =
                std::min(
                    rightRoot,
                    root);
        }
    }

    if (leftRoot >
            witnessTau +
                rootTolerance ||
        rightRoot <
            witnessTau -
                rootTolerance ||
        leftRoot >=
            rightRoot)
    {
        return result;
    }

    result.begin =
        leftRoot;

    result.end =
        rightRoot;

    result.valid =
        true;

    return result;
}


inline int
minimumDyadicDepthInsideInterval(
    const double normalizedTime,
    const double intervalBegin,
    const double intervalEnd,
    const int maxDepth,
    const double tolerance = 1.0e-12)
{
    if (!std::isfinite(normalizedTime) ||
        !std::isfinite(intervalBegin) ||
        !std::isfinite(intervalEnd) ||
        maxDepth < 0 ||
        normalizedTime <
            intervalBegin -
                tolerance ||
        normalizedTime >
            intervalEnd +
                tolerance ||
        intervalBegin <
            -tolerance ||
        intervalEnd >
            1.0 +
                tolerance ||
        intervalBegin >=
            intervalEnd)
    {
        return -1;
    }

    for (int depth = 0;
         depth <= maxDepth;
         ++depth)
    {
        const int leaf =
            bernsteinLeafForNormalizedTime(
                normalizedTime,
                depth);

        if (leaf < 0)
        {
            return -1;
        }

        const int leafCount =
            1 << depth;

        const double leafBegin =
            static_cast<double>(
                leaf) /
            static_cast<double>(
                leafCount);

        const double leafEnd =
            static_cast<double>(
                leaf + 1) /
            static_cast<double>(
                leafCount);

        if (leafBegin >=
                intervalBegin -
                    tolerance &&
            leafEnd <=
                intervalEnd +
                    tolerance)
        {
            return depth;
        }
    }

    return -1;
}

struct LazyBernsteinProjectionOptions
{
    double containment_tolerance_m =
        1.0e-6;

    // Use the same physical tolerance as the exact certificate.
    double bound_gap_tolerance_m =
        1.0e-6;

    // Numerical safeguards only.
    int max_adaptive_depth =
        16;

    int max_iterations =
        32;

    double qp_primal_tolerance =
        1.0e-10;

    double qp_dual_tolerance =
        1.0e-12;

    double duplicate_tolerance =
        1.0e-10;
};


struct LazyBernsteinIterationRecord
{
    int iteration = 0;

    int piece = -1;
    int face = -1;

    double tau = 0.0;

    bool violation_interval_valid =
        false;

    double violation_interval_begin =
        0.0;

    double violation_interval_end =
        1.0;

    int locality_depth =
        -1;

    double pre_violation_m =
        std::numeric_limits<double>::
            infinity();

    int depth = -1;
    int leaf = -1;

    double interval_begin = 0.0;
    double interval_end = 1.0;

    double bernstein_bound_m =
        std::numeric_limits<double>::
            infinity();

    double bound_gap_m =
        std::numeric_limits<double>::
            infinity();

    bool depth_saturated = false;

    int cut_rows = 0;
    int rows_added = 0;
    int active_rows = 0;

    bool qp_success = false;

    int qp_iterations = 0;

    int qp_working_set_size = 0;

    double qp_max_primal_violation =
        std::numeric_limits<double>::
            infinity();

    double qp_min_active_multiplier =
        std::numeric_limits<double>::
            infinity();

    double qp_equality_residual =
        std::numeric_limits<double>::
            infinity();

    double qp_ms = 0.0;

    double post_violation_m =
        std::numeric_limits<double>::
            infinity();

    int post_worst_piece = -1;
    int post_worst_face = -1;

    double post_worst_tau = 0.0;

    bool post_contained = false;
};


struct LazyBernsteinProjectionResult
{
    bool success = false;

    bool affine_map_valid = false;

    bool initial_certificate_valid = false;
    bool initial_contained = false;

    bool final_certificate_valid = false;
    bool final_contained = false;

    double initial_max_violation_m =
        std::numeric_limits<double>::
            infinity();

    double final_max_violation_m =
        std::numeric_limits<double>::
            infinity();

    int iterations = 0;

    int activated_cut_count = 0;

    int active_constraint_count = 0;

    int duplicate_cut_count = 0;

    int duplicate_row_count = 0;

    int depth_saturation_count = 0;

    int total_qp_iterations = 0;

    double qp_ms = 0.0;

    double certificate_ms = 0.0;

    double total_ms = 0.0;

    double correction_l2_m =
        std::numeric_limits<double>::
            infinity();

    double max_waypoint_displacement_m =
        std::numeric_limits<double>::
            infinity();

    std::vector<
        LazyBernsteinIterationRecord>
            iteration_records;
};


struct LazyBernsteinCutKey
{
    int piece = -1;
    int face = -1;
    int depth = -1;
    int leaf = -1;
};


inline LazyBernsteinProjectionResult
projectMincoToLazyBernsteinSfc(
    const Eigen::Matrix3d &headPVA,
    const Eigen::Matrix3d &tailPVA,
    const Eigen::Matrix3Xd &initialPoints,
    const Eigen::VectorXd &times,
    const std::vector<Eigen::MatrixX4d> &corridors,
    Trajectory<5> &projectedTrajectory,
    Eigen::Matrix3Xd &projectedPoints,
    const LazyBernsteinProjectionOptions &options =
        LazyBernsteinProjectionOptions())
{
    LazyBernsteinProjectionResult result;

    projectedTrajectory.clear();
    projectedPoints.resize(3, 0);

    const auto totalStarted =
        std::chrono::steady_clock::now();

    auto stampTotal =
        [&]()
        {
            result.total_ms =
                std::chrono::duration<
                    double,
                    std::milli>(
                        std::chrono::
                            steady_clock::now() -
                        totalStarted)
                    .count();
        };

    const int pieceCount =
        static_cast<int>(
            times.size());

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
        !times.allFinite() ||
        options.max_adaptive_depth < 0 ||
        options.max_adaptive_depth > 20 ||
        options.max_iterations <= 0)
    {
        stampTotal();
        return result;
    }

    MincoWaypointAffineMap affineMap;

    result.affine_map_valid =
        affineMap.build(
            headPVA,
            tailPVA,
            times);

    if (!result.affine_map_valid)
    {
        stampTotal();
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
            Trajectory<5> &trajectory)
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

            return
                trajectory.getPieceNum() ==
                    pieceCount;
        };

    if (!buildTrajectory(
            currentPoints,
            currentTrajectory))
    {
        stampTotal();
        return result;
    }

    auto certify =
        [&](const Trajectory<5> &trajectory)
        {
            const auto started =
                std::chrono::
                    steady_clock::now();

            const auto certificate =
                certifyMincoTrajectoryInCorridors(
                    trajectory,
                    corridors,
                    options
                        .containment_tolerance_m);

            result.certificate_ms +=
                std::chrono::duration<
                    double,
                    std::milli>(
                        std::chrono::
                            steady_clock::now() -
                        started)
                    .count();

            return certificate;
        };

    ExactTrajectoryCertificate certificate =
        certify(
            currentTrajectory);

    result.initial_certificate_valid =
        certificate.valid;

    result.initial_contained =
        certificate.contained;

    result.initial_max_violation_m =
        certificate.worst.violation_m;

    if (!certificate.valid)
    {
        stampTotal();
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
            certificate.worst.violation_m;

        result.correction_l2_m =
            0.0;

        result.max_waypoint_displacement_m =
            0.0;

        result.success =
            true;

        stampTotal();
        return result;
    }

    std::vector<Eigen::VectorXd>
        activeRows;

    std::vector<double>
        activeRhs;

    std::vector<LazyBernsteinCutKey>
        activatedCuts;

    for (int iteration = 0;
         iteration <
             options.max_iterations;
         ++iteration)
    {
        if (!certificate.worst.valid ||
            certificate.worst.piece < 0 ||
            certificate.worst.face < 0 ||
            !std::isfinite(
                certificate.worst
                    .normalized_time) ||
            !std::isfinite(
                certificate.worst
                    .violation_m))
        {
            stampTotal();
            return result;
        }

        LazyBernsteinIterationRecord record;

        record.iteration =
            iteration + 1;

        record.piece =
            certificate.worst.piece;

        record.face =
            certificate.worst.face;

        record.tau =
            certificate.worst
                .normalized_time;

        record.pre_violation_m =
            certificate.worst
                .violation_m;

        const int witnessPiece =
            certificate.worst.piece;

        const int witnessFace =
            certificate.worst.face;

        if (witnessPiece < 0 ||
            witnessPiece >=
                currentTrajectory
                    .getPieceNum() ||
            witnessFace < 0 ||
            witnessFace >=
                corridors[
                    witnessPiece]
                    .rows())
        {
            stampTotal();
            return result;
        }

        const auto &witnessPoly =
            corridors[
                witnessPiece];

        const Eigen::Vector3d
            witnessNormal =
                witnessPoly
                    .block<1, 3>(
                        witnessFace,
                        0)
                    .transpose();

        const double
            witnessPlaneOffset =
                witnessPoly(
                    witnessFace,
                    3);

        const auto violationInterval =
            exactFaceViolationInterval(
                currentTrajectory[
                    witnessPiece],
                witnessNormal,
                witnessPlaneOffset,
                certificate.worst
                    .normalized_time);

        record.violation_interval_valid =
            violationInterval.valid;

        record.violation_interval_begin =
            violationInterval.begin;

        record.violation_interval_end =
            violationInterval.end;

        if (!violationInterval.valid)
        {
            stampTotal();
            return result;
        }

        const int localityDepth =
            minimumDyadicDepthInsideInterval(
                certificate.worst
                    .normalized_time,
                violationInterval.begin,
                violationInterval.end,
                options
                    .max_adaptive_depth);

        record.locality_depth =
            localityDepth;

        if (localityDepth < 0)
        {
            stampTotal();
            return result;
        }

        int selectedDepth =
            -1;

        double selectedBoundM =
            std::numeric_limits<double>::
                infinity();

        double selectedGapM =
            std::numeric_limits<double>::
                infinity();

        int selectedLeaf =
            -1;

        double selectedBegin =
            0.0;

        double selectedEnd =
            1.0;

        for (int depth =
                 localityDepth;
             depth <=
                 options.max_adaptive_depth;
             ++depth)
        {
            double boundM =
                -std::numeric_limits<double>::
                    infinity();

            int leafId =
                -1;

            double intervalBegin =
                0.0;

            double intervalEnd =
                1.0;

            if (!evaluateLocalBernsteinFaceBoundM(
                    affineMap,
                    corridors,
                    currentPoints,
                    certificate.worst.piece,
                    certificate.worst.face,
                    certificate.worst
                        .normalized_time,
                    depth,
                    boundM,
                    leafId,
                    intervalBegin,
                    intervalEnd))
            {
                stampTotal();
                return result;
            }

            // Bernstein hull must upper-bound the exact
            // maximum on the interval containing tau*.
            if (boundM + 1.0e-9 <
                certificate.worst
                    .violation_m)
            {
                stampTotal();
                return result;
            }

            const double gapM =
                std::max(
                    0.0,
                    boundM -
                        certificate.worst
                            .violation_m);

            selectedDepth =
                depth;

            selectedBoundM =
                boundM;

            selectedGapM =
                gapM;

            selectedLeaf =
                leafId;

            selectedBegin =
                intervalBegin;

            selectedEnd =
                intervalEnd;

            if (gapM <=
                options
                    .bound_gap_tolerance_m)
            {
                break;
            }
        }

        if (selectedDepth < 0)
        {
            stampTotal();
            return result;
        }

        record.depth =
            selectedDepth;

        record.leaf =
            selectedLeaf;

        record.interval_begin =
            selectedBegin;

        record.interval_end =
            selectedEnd;

        record.bernstein_bound_m =
            selectedBoundM;

        record.bound_gap_m =
            selectedGapM;

        record.depth_saturated =
            selectedDepth ==
                options.max_adaptive_depth &&
            selectedGapM >
                options
                    .bound_gap_tolerance_m;

        if (record.depth_saturated)
        {
            ++result
                  .depth_saturation_count;
        }

        const LocalBernsteinCut localCut =
            buildLocalBernsteinCut(
                affineMap,
                corridors,
                certificate.worst.piece,
                certificate.worst.face,
                certificate.worst
                    .normalized_time,
                selectedDepth);

        if (!localCut.valid ||
            localCut.fixed_infeasible ||
            localCut.A.rows() <= 0 ||
            localCut.A.rows() > 6 ||
            localCut.A.cols() !=
                z0.size())
        {
            stampTotal();
            return result;
        }

        record.cut_rows =
            localCut.A.rows();

        LazyBernsteinCutKey key;

        key.piece =
            localCut.piece;

        key.face =
            localCut.face;

        key.depth =
            localCut.depth;

        key.leaf =
            localCut.leaf;

        bool duplicateCut =
            false;

        for (const auto &oldKey :
             activatedCuts)
        {
            if (oldKey.piece ==
                    key.piece &&
                oldKey.face ==
                    key.face &&
                oldKey.depth ==
                    key.depth &&
                oldKey.leaf ==
                    key.leaf)
            {
                duplicateCut =
                    true;

                break;
            }
        }

        if (duplicateCut)
        {
            ++result
                  .duplicate_cut_count;

            result.iteration_records
                .push_back(
                    record);

            stampTotal();
            return result;
        }

        int rowsAdded =
            0;

        for (int rowId = 0;
             rowId <
                 localCut.A.rows();
             ++rowId)
        {
            const Eigen::VectorXd row =
                localCut.A
                    .row(rowId)
                    .transpose();

            const double rhs =
                localCut.b(rowId);

            bool duplicateRow =
                false;

            for (int oldId = 0;
                 oldId <
                     static_cast<int>(
                         activeRows.size());
                 ++oldId)
            {
                if ((activeRows[oldId] -
                     row)
                            .norm() <=
                        options
                            .duplicate_tolerance &&
                    std::abs(
                        activeRhs[oldId] -
                        rhs) <=
                        options
                            .duplicate_tolerance)
                {
                    duplicateRow =
                        true;

                    break;
                }
            }

            if (duplicateRow)
            {
                ++result
                      .duplicate_row_count;

                continue;
            }

            activeRows.push_back(
                row);

            activeRhs.push_back(
                rhs);

            ++rowsAdded;
        }

        record.rows_added =
            rowsAdded;

        if (rowsAdded <= 0)
        {
            result.iteration_records
                .push_back(
                    record);

            stampTotal();
            return result;
        }

        activatedCuts.push_back(
            key);

        ++result
              .activated_cut_count;

        record.active_rows =
            static_cast<int>(
                activeRows.size());

        const auto qpStarted =
            std::chrono::
                steady_clock::now();

        const ActiveSetProjectionResult qp =
            solveEuclideanHalfspaceProjectionActiveSet(
                z0,
                activeRows,
                activeRhs,
                options.qp_primal_tolerance,
                options.qp_dual_tolerance);

        record.qp_ms =
            std::chrono::duration<
                double,
                std::milli>(
                    std::chrono::
                        steady_clock::now() -
                    qpStarted)
                .count();

        result.qp_ms +=
            record.qp_ms;

        record.qp_success =
            qp.success &&
            qp.solution.allFinite();

        record.qp_iterations =
            qp.iterations;

        record.qp_working_set_size =
            qp.working_set_size;

        record.qp_max_primal_violation =
            qp.max_primal_violation;

        record.qp_min_active_multiplier =
            qp.min_active_multiplier;

        record.qp_equality_residual =
            qp.equality_residual;

        result.total_qp_iterations +=
            qp.iterations;

        if (!record.qp_success)
        {
            result.iteration_records
                .push_back(
                    record);

            result.iterations =
                iteration + 1;

            result.active_constraint_count =
                activeRows.size();

            stampTotal();
            return result;
        }

        z =
            qp.solution;

        if (!unflattenMincoWaypoints(
                z,
                currentPoints))
        {
            stampTotal();
            return result;
        }

        if (!buildTrajectory(
                currentPoints,
                currentTrajectory))
        {
            stampTotal();
            return result;
        }

        certificate =
            certify(
                currentTrajectory);

        if (!certificate.valid)
        {
            result.iteration_records
                .push_back(
                    record);

            stampTotal();
            return result;
        }

        record.post_violation_m =
            certificate.worst
                .violation_m;

        record.post_worst_piece =
            certificate.worst
                .piece;

        record.post_worst_face =
            certificate.worst
                .face;

        record.post_worst_tau =
            certificate.worst
                .normalized_time;

        record.post_contained =
            certificate.contained;

        result.iteration_records
            .push_back(
                record);

        result.iterations =
            iteration + 1;

        result.active_constraint_count =
            activeRows.size();

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
                (currentPoints
                     .col(waypointId) -
                 initialPoints
                     .col(waypointId))
                    .norm());
    }

    result.success =
        result.final_certificate_valid &&
        result.final_contained;

    stampTotal();

    return result;
}

} // namespace traj_relevant

#endif