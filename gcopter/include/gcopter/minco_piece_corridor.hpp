#ifndef MINCO_PIECE_CORRIDOR_HPP
#define MINCO_PIECE_CORRIDOR_HPP

#include "minco_support.hpp"

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

struct MincoPieceCorridorOptions
{
    // Extra free space in the most useful CSGN direction.
    double max_extra_radius =
        3.0;

    // Least useful CSGN direction receives beta * max_extra_radius.
    double min_extra_ratio =
        0.25;

    // This is an overlap / protected-trajectory margin.
    //
    // IMPORTANT:
    // The obstacle point cloud is already generated from the dilated
    // voxel map, therefore this must not silently duplicate the robot
    // collision radius.
    double overlap_radius =
        0.01;

    bool metric_enabled =
        false;

    Eigen::Matrix3d deformation_utility =
        Eigen::Matrix3d::Identity();

    double epsilon =
        1.0e-6;

    double root_tolerance =
        1.0e-10;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MincoPieceCorridorDiagnostics
{
    int input_obstacle_count =
        0;

    int local_obstacle_count =
        0;

    int domain_face_count =
        0;

    int candidate_count =
        0;

    int rejected_candidate_count =
        0;

    int greedy_obstacle_face_count =
        0;

    int selected_obstacle_face_count =
        0;

    int redundancy_removed =
        0;

    int unresolved_obstacle_count =
        0;

    int projection_fallback_attempt_count =
        0;

    int projection_fallback_success_count =
        0;

    int projection_fallback_iteration_count =
        0;

    double projection_fallback_max_margin_m =
        0.0;

    int unresolved_projection_valid_count =
        0;

    int unresolved_certified_separable_count =
        0;

    int unresolved_projection_ambiguous_count =
        0;

    int first_unresolved_obstacle =
        -1;

    bool first_unresolved_projection_valid =
        false;

    bool first_unresolved_projection_converged =
        false;

    bool first_unresolved_certified_separable =
        false;

    double first_unresolved_metric_distance_squared =
        std::numeric_limits<double>::infinity();

    double first_unresolved_metric_distance_lower_bound_squared =
        0.0;

    double first_unresolved_euclidean_distance =
        std::numeric_limits<double>::infinity();

    double first_unresolved_separation_margin_m =
        -std::numeric_limits<double>::infinity();

    double first_unresolved_fw_gap =
        std::numeric_limits<double>::infinity();

    int first_unresolved_fw_iterations =
        0;

    int total_face_count =
        0;

    int max_closest_stationary_points =
        0;

    bool metric_valid =
        false;

    bool anisotropic_domain =
        false;

    bool trajectory_contained =
        false;

    bool endpoint_ball_guaranteed =
        false;

    bool safety_verified =
        false;

    Eigen::Vector3d utility_eigenvalues =
        Eigen::Vector3d::Ones();

    Eigen::Vector3d extra_radii =
        Eigen::Vector3d::Zero();

    double mean_metric_damage =
        0.0;

    double min_metric_damage =
        0.0;

    double max_metric_damage =
        0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MincoCandidateFace
{
    Eigen::Vector4d plane =
        Eigen::Vector4d::Zero();

    std::vector<int>
        covered_obstacles;

    int source_obstacle =
        -1;

    double metric_damage =
        0.0;

    double separation_margin =
        0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MincoCandidateFaces =
    std::vector<
        MincoCandidateFace,
        Eigen::aligned_allocator<
            MincoCandidateFace>>;


// ================================================================
// Build one convex corridor directly around one MINCO polynomial
// piece.
//
// Protected set:
//
//   K = conv(
//         Gamma
//         union B(p(0), r_overlap)
//         union B(p(T), r_overlap)).
//
// Therefore:
//
//   Gamma subset K
//
// and adjacent MINCO pieces sharing junction q satisfy
//
//   B(q, r_overlap)
//       subset P_i intersection P_{i+1}.
//
// The overlap radius is NOT applied along the complete trajectory,
// because obstacle clearance has already been handled by the dilated
// voxel map.
//
// Candidate normal for obstacle o:
//
//   q* = argmin_t
//          (o-p(t))^T W (o-p(t))
//
//   n  = W(o-q*)
//
// with
//
//   W = S^{-1}.
//
// The closest-point normal is only a candidate generator.
// Safety never relies on local projection optimality: exact
// directional support is used afterwards to certify that the
// complete polynomial lies on the protected side.
// ================================================================
template <int D>
inline bool buildCompactMincoPiecePolytope(
    const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector3d &lowCorner,
    const Eigen::Vector3d &highCorner,
    const Piece<D> &piece,
    const MincoPieceCorridorOptions &options,
    Eigen::MatrixX4d &hPoly,
    MincoPieceCorridorDiagnostics *diagnostics =
        nullptr)
{
    MincoPieceCorridorDiagnostics localDiagnostics;

    localDiagnostics.input_obstacle_count =
        static_cast<int>(
            points.size());

    const double epsilon =
        std::max(
            options.epsilon,
            1.0e-12);

    const double rootTolerance =
        std::max(
            options.root_tolerance,
            1.0e-12);

    const double overlapRadius =
        std::max(
            0.0,
            options.overlap_radius);

    const double maxExtraRadius =
        std::max(
            0.0,
            options.max_extra_radius);

    const double minExtraRatio =
        std::min(
            1.0,
            std::max(
                0.0,
                options.min_extra_ratio));

    // ------------------------------------------------------------
    // Validate global-map containment of the protected trajectory.
    // ------------------------------------------------------------
    for (int axis = 0;
         axis < 3;
         ++axis)
    {
        const Eigen::Vector3d direction =
            Eigen::Vector3d::Unit(
                axis);

        const auto positiveSupport =
            protectedMincoPieceDirectionalSupport(
                piece,
                direction,
                overlapRadius,
                rootTolerance);

        const auto negativeSupport =
            protectedMincoPieceDirectionalSupport(
                piece,
                -direction,
                overlapRadius,
                rootTolerance);

        if (!positiveSupport.valid ||
            !negativeSupport.valid)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }

        if (positiveSupport.support >
                highCorner(axis) +
                    epsilon ||
            negativeSupport.support >
                -lowCorner(axis) +
                    epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }
    }

    // ------------------------------------------------------------
    // Validate CSGN deformation utility:
    //
    //   S = U diag(mu) U^T.
    //
    // Large mu means useful/easy MINCO deformation.
    // ------------------------------------------------------------
    Eigen::Matrix3d utility =
        0.5 *
        (options.deformation_utility +
         options.deformation_utility.transpose());

    Eigen::Matrix3d eigenvectors =
        Eigen::Matrix3d::Identity();

    Eigen::Vector3d eigenvalues =
        Eigen::Vector3d::Ones();

    if (options.metric_enabled &&
        utility.allFinite())
    {
        Eigen::SelfAdjointEigenSolver<
            Eigen::Matrix3d>
            solver(
                utility);

        if (solver.info() ==
                Eigen::Success &&
            solver.eigenvalues()
                    .minCoeff() >
                epsilon)
        {
            localDiagnostics.metric_valid =
                true;

            eigenvectors =
                solver.eigenvectors();

            eigenvalues =
                solver.eigenvalues();
        }
    }

    if (!localDiagnostics.metric_valid)
    {
        utility.setIdentity();
        eigenvectors.setIdentity();
        eigenvalues.setOnes();
    }

    localDiagnostics.utility_eigenvalues =
        eigenvalues;

    Eigen::Matrix3d inverseUtility =
        Eigen::Matrix3d::Identity();

    if (localDiagnostics.metric_valid)
    {
        inverseUtility =
            eigenvectors *
            eigenvalues
                .cwiseInverse()
                .asDiagonal() *
            eigenvectors.transpose();
    }

    // ------------------------------------------------------------
    // CSGN spectrum -> useful directional expansion.
    // ------------------------------------------------------------
    Eigen::Vector3d normalizedUtility =
        Eigen::Vector3d::Ones();

    if (localDiagnostics.metric_valid)
    {
        const Eigen::Vector3d logEigenvalues =
            eigenvalues.array().log();

        const double logMin =
            logEigenvalues.minCoeff();

        const double logMax =
            logEigenvalues.maxCoeff();

        const double logSpan =
            logMax -
            logMin;

        if (logSpan >
            epsilon)
        {
            normalizedUtility =
                (logEigenvalues.array() -
                 logMin) /
                logSpan;

            localDiagnostics.anisotropic_domain =
                true;
        }
    }

    const Eigen::Vector3d extraRadii =
        maxExtraRadius *
        (minExtraRatio *
             Eigen::Vector3d::Ones() +
         (1.0 -
          minExtraRatio) *
             normalizedUtility);

    localDiagnostics.extra_radii =
        extraRadii;

    // ------------------------------------------------------------
    // Exact trajectory-relevant oriented domain.
    //
    // For eigenvector u_j:
    //
    //   upper_j =
    //       h_Gamma(u_j) + r + e_j
    //
    //   lower_j =
    //      -h_Gamma(-u_j) - r - e_j.
    //
    // This is trajectory-native: no RRT segment and no midpoint box
    // approximation are used.
    // ------------------------------------------------------------
    std::vector<
        Eigen::Vector4d,
        Eigen::aligned_allocator<
            Eigen::Vector4d>>
        fixedPlanes;

    fixedPlanes.reserve(
        12);

    Eigen::Vector3d domainCenterCoordinates =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d domainHalfWidths =
        Eigen::Vector3d::Zero();

    for (int directionId = 0;
         directionId < 3;
         ++directionId)
    {
        const Eigen::Vector3d direction =
            eigenvectors.col(
                directionId);

        const auto positiveSupport =
            protectedMincoPieceDirectionalSupport(
                piece,
                direction,
                overlapRadius,
                rootTolerance);

        const auto negativeSupport =
            protectedMincoPieceDirectionalSupport(
                piece,
                -direction,
                overlapRadius,
                rootTolerance);

        if (!positiveSupport.valid ||
            !negativeSupport.valid)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }

        const double upperCoordinate =
            positiveSupport.support +
            extraRadii(directionId);

        const double lowerCoordinate =
            -negativeSupport.support -
            extraRadii(directionId);

        domainCenterCoordinates(
            directionId) =
            0.5 *
            (upperCoordinate +
             lowerCoordinate);

        domainHalfWidths(
            directionId) =
            0.5 *
            (upperCoordinate -
             lowerCoordinate);

        Eigen::Vector4d upperPlane =
            Eigen::Vector4d::Zero();

        upperPlane.head<3>() =
            direction;

        upperPlane(3) =
            -upperCoordinate;

        Eigen::Vector4d lowerPlane =
            Eigen::Vector4d::Zero();

        lowerPlane.head<3>() =
            -direction;

        lowerPlane(3) =
            -negativeSupport.support -
            extraRadii(directionId);

        fixedPlanes.push_back(
            upperPlane);

        fixedPlanes.push_back(
            lowerPlane);
    }

    // ------------------------------------------------------------
    // Clip the oriented box against map bounds only if necessary.
    // ------------------------------------------------------------
    const Eigen::Vector3d domainCenter =
        eigenvectors *
        domainCenterCoordinates;

    for (int axis = 0;
         axis < 3;
         ++axis)
    {
        double worldHalfWidth =
            0.0;

        for (int directionId = 0;
             directionId < 3;
             ++directionId)
        {
            worldHalfWidth +=
                std::abs(
                    eigenvectors(
                        axis,
                        directionId)) *
                domainHalfWidths(
                    directionId);
        }

        const double domainHigh =
            domainCenter(axis) +
            worldHalfWidth;

        const double domainLow =
            domainCenter(axis) -
            worldHalfWidth;

        if (domainHigh >
            highCorner(axis) +
                epsilon)
        {
            Eigen::Vector4d plane =
                Eigen::Vector4d::Zero();

            plane(axis) =
                1.0;

            plane(3) =
                -highCorner(axis);

            fixedPlanes.push_back(
                plane);
        }

        if (domainLow <
            lowCorner(axis) -
                epsilon)
        {
            Eigen::Vector4d plane =
                Eigen::Vector4d::Zero();

            plane(axis) =
                -1.0;

            plane(3) =
                lowCorner(axis);

            fixedPlanes.push_back(
                plane);
        }
    }

    localDiagnostics.domain_face_count =
        static_cast<int>(
            fixedPlanes.size());

    Eigen::MatrixX4d fixedH(
        static_cast<int>(
            fixedPlanes.size()),
        4);

    for (int faceId = 0;
         faceId <
             static_cast<int>(
                 fixedPlanes.size());
         ++faceId)
    {
        fixedH.row(
            faceId) =
            fixedPlanes[
                faceId]
                .transpose();
    }

    // ------------------------------------------------------------
    // Obstacles outside the trajectory-relevant domain are already
    // excluded by one of the fixed domain halfspaces.
    // ------------------------------------------------------------
    std::vector<
        Eigen::Vector3d,
        Eigen::aligned_allocator<
            Eigen::Vector3d>>
        localObstacles;

    localObstacles.reserve(
        points.size());

    for (const Eigen::Vector3d &point :
         points)
    {
        const Eigen::Vector4d pointH(
            point(0),
            point(1),
            point(2),
            1.0);

        if ((fixedH *
             pointH)
                .maxCoeff() <=
            epsilon)
        {
            localObstacles.push_back(
                point);
        }
    }

    const int obstacleCount =
        static_cast<int>(
            localObstacles.size());

    localDiagnostics.local_obstacle_count =
        obstacleCount;

    // ------------------------------------------------------------
    // Candidate generation.
    // ------------------------------------------------------------
    MincoCandidateFaces candidates;

    candidates.reserve(
        obstacleCount);

    for (int obstacleId = 0;
         obstacleId <
             obstacleCount;
         ++obstacleId)
    {
        const Eigen::Vector3d &obstacle =
            localObstacles[
                obstacleId];

        Eigen::Vector3d normal =
            Eigen::Vector3d::Zero();

        bool normalAvailable =
            false;

        // ============================================================
        // Stage 1:
        // Fast candidate from closest point on the actual MINCO curve.
        // ============================================================
        const auto closest =
            exactMincoMetricClosestPoint(
                piece,
                obstacle,
                inverseUtility,
                rootTolerance);

        if (closest.valid)
        {
            localDiagnostics
                .max_closest_stationary_points =
                std::max(
                    localDiagnostics
                        .max_closest_stationary_points,
                    closest
                        .stationary_point_count);

            normal =
                inverseUtility *
                (obstacle -
                 closest.point);

            const double normalNorm =
                normal.norm();

            if (normal.allFinite() &&
                std::isfinite(normalNorm) &&
                normalNorm >
                    epsilon)
            {
                normal /=
                    normalNorm;

                normalAvailable =
                    true;
            }
        }

        // ------------------------------------------------------------
        // Check whether the fast curve-derived normal is actually a
        // separator of the COMPLETE convex protected set.
        // ------------------------------------------------------------
        bool separatorCertified =
            false;

        double protectedSupport =
            0.0;

        double obstacleSupport =
            0.0;

        double separationMargin =
            -std::numeric_limits<double>::
                infinity();

        if (normalAvailable)
        {
            const auto support =
                protectedMincoPieceDirectionalSupport(
                    piece,
                    normal,
                    overlapRadius,
                    rootTolerance);

            if (support.valid)
            {
                protectedSupport =
                    support.support;

                obstacleSupport =
                    normal.dot(
                        obstacle);

                separationMargin =
                    obstacleSupport -
                    protectedSupport;

                separatorCertified =
                    std::isfinite(
                        separationMargin) &&
                    separationMargin >
                        2.0 *
                            epsilon;
            }
        }

        // ============================================================
        // Stage 2:
        // If projection onto the non-convex polynomial curve does not
        // provide a legal separator, refine against the actual convex
        // protected set K.
        //
        // IMPORTANT:
        // We do NOT require Frank-Wolfe convergence.
        // Once exact support certifies:
        //
        //     n^T o - h_K(n) > eps
        //
        // a legal separating face has already been found.
        // ============================================================
        if (!separatorCertified)
        {
            ++localDiagnostics
                  .projection_fallback_attempt_count;

            const auto projection =
                projectOntoProtectedMincoSet(
                    piece,
                    obstacle,
                    inverseUtility,
                    overlapRadius,
                    512,
                    1.0e-10,
                    1.0e-10,
                    epsilon,
                    rootTolerance);

            localDiagnostics
                .projection_fallback_iteration_count +=
                projection.iterations;

            if (projection.valid &&
                projection.certified_separable &&
                projection.separating_normal.allFinite())
            {
                normal =
                    projection
                        .separating_normal;

                const double normalNorm =
                    normal.norm();

                if (std::isfinite(
                        normalNorm) &&
                    normalNorm >
                        epsilon)
                {
                    normal /=
                        normalNorm;

                    // Never rely only on values saved by the numerical
                    // projection routine. Re-certify the final face here.
                    const auto support =
                        protectedMincoPieceDirectionalSupport(
                            piece,
                            normal,
                            overlapRadius,
                            rootTolerance);

                    if (support.valid)
                    {
                        protectedSupport =
                            support.support;

                        obstacleSupport =
                            normal.dot(
                                obstacle);

                        separationMargin =
                            obstacleSupport -
                            protectedSupport;

                        separatorCertified =
                            std::isfinite(
                                separationMargin) &&
                            separationMargin >
                                2.0 *
                                    epsilon;

                        if (separatorCertified)
                        {
                            ++localDiagnostics
                                  .projection_fallback_success_count;

                            localDiagnostics
                                .projection_fallback_max_margin_m =
                                std::max(
                                    localDiagnostics
                                        .projection_fallback_max_margin_m,
                                    separationMargin);
                        }
                    }
                }
            }
        }

        if (!separatorCertified)
        {
            ++localDiagnostics
                  .rejected_candidate_count;

            continue;
        }

        // Mid-plane leaves nonzero clearance to both protected
        // trajectory and generating obstacle.
        const double threshold =
            0.5 *
            (protectedSupport +
             obstacleSupport);

        MincoCandidateFace candidate;

        candidate.source_obstacle =
            obstacleId;

        candidate.plane.head<3>() =
            normal;

        candidate.plane(3) =
            -threshold;

        candidate.separation_margin =
            separationMargin;

        const double rayleigh =
            normal.dot(
                utility *
                normal);

        if (std::isfinite(
                rayleigh) &&
            rayleigh >
                epsilon)
        {
            candidate.metric_damage =
                0.5 *
                std::log(
                    rayleigh);
        }

        // Batch set-cover relation.
        for (int pointId = 0;
             pointId <
                 obstacleCount;
             ++pointId)
        {
            const double violation =
                normal.dot(
                    localObstacles[
                        pointId]) -
                threshold;

            if (violation >
                epsilon)
            {
                candidate
                    .covered_obstacles
                    .push_back(
                        pointId);
            }
        }

        if (candidate
                .covered_obstacles
                .empty())
        {
            ++localDiagnostics
                  .rejected_candidate_count;

            continue;
        }

        candidates.emplace_back(
            std::move(
                candidate));
    }

    localDiagnostics.candidate_count =
        static_cast<int>(
            candidates.size());

    if (obstacleCount > 0 &&
        candidates.empty())
    {
        localDiagnostics
            .unresolved_obstacle_count =
            obstacleCount;

        if (diagnostics != nullptr)
        {
            *diagnostics =
                localDiagnostics;
        }

        return false;
    }

    // ------------------------------------------------------------
    // Batch greedy minimum-face cover.
    //
    // Lexicographic objective:
    //
    // 1. maximize newly excluded obstacles;
    // 2. minimize CSGN directional damage;
    // 3. maximize trajectory-obstacle separation.
    // ------------------------------------------------------------
    std::vector<unsigned char>
        unresolved(
            obstacleCount,
            1);

    std::vector<unsigned char>
        selected(
            candidates.size(),
            0);

    int unresolvedCount =
        obstacleCount;

    std::vector<int>
        selectedCandidateIds;

    while (unresolvedCount >
           0)
    {
        int bestCandidate =
            -1;

        int bestGain =
            0;

        double bestDamage =
            std::numeric_limits<double>::
                infinity();

        double bestMargin =
            -std::numeric_limits<double>::
                infinity();

        for (int candidateId = 0;
             candidateId <
                 static_cast<int>(
                     candidates.size());
             ++candidateId)
        {
            if (selected[
                    candidateId])
            {
                continue;
            }

            int gain =
                0;

            for (const int obstacleId :
                 candidates[
                     candidateId]
                     .covered_obstacles)
            {
                if (unresolved[
                        obstacleId])
                {
                    ++gain;
                }
            }

            if (gain <=
                0)
            {
                continue;
            }

            const double damage =
                candidates[
                    candidateId]
                    .metric_damage;

            const double margin =
                candidates[
                    candidateId]
                    .separation_margin;

            const bool prefer =
                gain >
                    bestGain ||

                (gain ==
                     bestGain &&
                 damage <
                     bestDamage -
                         epsilon) ||

                (gain ==
                     bestGain &&
                 std::abs(
                     damage -
                     bestDamage) <=
                     epsilon &&
                 margin >
                     bestMargin);

            if (prefer)
            {
                bestCandidate =
                    candidateId;

                bestGain =
                    gain;

                bestDamage =
                    damage;

                bestMargin =
                    margin;
            }
        }

        if (bestCandidate <
                0 ||
            bestGain <=
                0)
        {
            localDiagnostics
                .unresolved_obstacle_count =
                unresolvedCount;

            bool firstUnresolvedStored =
                false;

            for (int obstacleId = 0;
                 obstacleId <
                     obstacleCount;
                 ++obstacleId)
            {
                if (!unresolved[
                        obstacleId])
                {
                    continue;
                }

                const auto projection =
                    projectOntoProtectedMincoSet(
                        piece,
                        localObstacles[
                            obstacleId],
                        inverseUtility,
                        overlapRadius,
                        128,
                        1.0e-10,
                        1.0e-10,
                        epsilon,
                        rootTolerance);

                if (projection.valid)
                {
                    ++localDiagnostics
                          .unresolved_projection_valid_count;

                    if (projection
                            .certified_separable)
                    {
                        ++localDiagnostics
                              .unresolved_certified_separable_count;
                    }
                    else
                    {
                        ++localDiagnostics
                              .unresolved_projection_ambiguous_count;
                    }
                }
                else
                {
                    ++localDiagnostics
                          .unresolved_projection_ambiguous_count;
                }

                if (!firstUnresolvedStored)
                {
                    firstUnresolvedStored =
                        true;

                    localDiagnostics
                        .first_unresolved_obstacle =
                        obstacleId;

                    localDiagnostics
                        .first_unresolved_projection_valid =
                        projection.valid;

                    localDiagnostics
                        .first_unresolved_projection_converged =
                        projection.converged;

                    localDiagnostics
                        .first_unresolved_certified_separable =
                        projection.certified_separable;

                    localDiagnostics
                        .first_unresolved_metric_distance_squared =
                        projection.metric_distance_squared;

                    localDiagnostics
                        .first_unresolved_metric_distance_lower_bound_squared =
                        projection
                            .metric_distance_squared_lower_bound;

                    localDiagnostics
                        .first_unresolved_euclidean_distance =
                        projection.euclidean_distance;

                    localDiagnostics
                        .first_unresolved_separation_margin_m =
                        projection
                            .certified_separation_margin_m;

                    localDiagnostics
                        .first_unresolved_fw_gap =
                        projection.frank_wolfe_gap;

                    localDiagnostics
                        .first_unresolved_fw_iterations =
                        projection.iterations;

                    const int projectionIterationBudgets[] = {128, 512, 2048, 8192};
                    for (const int iterationBudget : projectionIterationBudgets)
                    {
                        const auto projectionSweep =
                            projectOntoProtectedMincoSet(
                                piece,
                                localObstacles[obstacleId],
                                inverseUtility,
                                overlapRadius,
                                iterationBudget,
                                1.0e-10,
                                1.0e-10,
                                epsilon,
                                rootTolerance);
                            
                        ROS_WARN_STREAM(
                            "TF_MINCO_PROJECTION_SWEEP "
                            << "obstacle=" << obstacleId
                            << " max_iters=" << iterationBudget
                            << " valid=" << projectionSweep.valid
                            << " converged=" << projectionSweep.converged
                            << " iters=" << projectionSweep.iterations
                            << " metric_d2=" << projectionSweep.metric_distance_squared
                            << " metric_d2_lb=" << projectionSweep.metric_distance_squared_lower_bound
                            << " euclidean_d=" << projectionSweep.euclidean_distance
                            << " sep_margin_m=" << projectionSweep.certified_separation_margin_m
                            << " separable=" << projectionSweep.certified_separable
                            << " fw_gap=" << projectionSweep.frank_wolfe_gap);
                    }
                }
            }

            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }

        selected[
            bestCandidate] =
            1;

        selectedCandidateIds
            .push_back(
                bestCandidate);

        for (const int obstacleId :
             candidates[
                 bestCandidate]
                 .covered_obstacles)
        {
            if (unresolved[
                    obstacleId])
            {
                unresolved[
                    obstacleId] =
                    0;

                --unresolvedCount;
            }
        }
    }

    localDiagnostics
        .greedy_obstacle_face_count =
        static_cast<int>(
            selectedCandidateIds.size());

    // ------------------------------------------------------------
    // Reverse-delete redundant selected faces.
    // ------------------------------------------------------------
    std::vector<int>
        coverCount(
            obstacleCount,
            0);

    for (const int candidateId :
         selectedCandidateIds)
    {
        for (const int obstacleId :
             candidates[
                 candidateId]
                 .covered_obstacles)
        {
            ++coverCount[
                obstacleId];
        }
    }

    std::vector<int>
        removalOrder =
            selectedCandidateIds;

    std::sort(
        removalOrder.begin(),
        removalOrder.end(),
        [&](const int lhs,
            const int rhs)
        {
            const double lhsDamage =
                candidates[lhs]
                    .metric_damage;

            const double rhsDamage =
                candidates[rhs]
                    .metric_damage;

            if (std::abs(
                    lhsDamage -
                    rhsDamage) >
                epsilon)
            {
                return
                    lhsDamage >
                    rhsDamage;
            }

            return
                candidates[lhs]
                    .covered_obstacles
                    .size() <
                candidates[rhs]
                    .covered_obstacles
                    .size();
        });

    for (const int candidateId :
         removalOrder)
    {
        if (!selected[
                candidateId])
        {
            continue;
        }

        bool removable =
            true;

        for (const int obstacleId :
             candidates[
                 candidateId]
                 .covered_obstacles)
        {
            if (coverCount[
                    obstacleId] <=
                1)
            {
                removable =
                    false;

                break;
            }
        }

        if (!removable)
        {
            continue;
        }

        selected[
            candidateId] =
            0;

        ++localDiagnostics
              .redundancy_removed;

        for (const int obstacleId :
             candidates[
                 candidateId]
                 .covered_obstacles)
        {
            --coverCount[
                obstacleId];
        }
    }

    int selectedObstacleFaceCount =
        0;

    for (const unsigned char flag :
         selected)
    {
        selectedObstacleFaceCount +=
            flag
                ? 1
                : 0;
    }

    localDiagnostics
        .selected_obstacle_face_count =
        selectedObstacleFaceCount;

    // ------------------------------------------------------------
    // Assemble final polytope.
    // ------------------------------------------------------------
    hPoly.resize(
        fixedH.rows() +
            selectedObstacleFaceCount,
        4);

    hPoly.topRows(
        fixedH.rows()) =
        fixedH;

    int outputRow =
        fixedH.rows();

    double damageSum =
        0.0;

    double minDamage =
        std::numeric_limits<double>::
            infinity();

    double maxDamage =
        -std::numeric_limits<double>::
            infinity();

    for (int candidateId = 0;
         candidateId <
             static_cast<int>(
                 candidates.size());
         ++candidateId)
    {
        if (!selected[
                candidateId])
        {
            continue;
        }

        hPoly.row(
            outputRow++) =
            candidates[
                candidateId]
                .plane
                .transpose();

        const double damage =
            candidates[
                candidateId]
                .metric_damage;

        damageSum +=
            damage;

        minDamage =
            std::min(
                minDamage,
                damage);

        maxDamage =
            std::max(
                maxDamage,
                damage);
    }

    localDiagnostics.total_face_count =
        static_cast<int>(
            hPoly.rows());

    if (selectedObstacleFaceCount >
        0)
    {
        localDiagnostics.mean_metric_damage =
            damageSum /
            static_cast<double>(
                selectedObstacleFaceCount);

        localDiagnostics.min_metric_damage =
            minDamage;

        localDiagnostics.max_metric_damage =
            maxDamage;
    }

    // ------------------------------------------------------------
    // Continuous-time trajectory containment certificate.
    // ------------------------------------------------------------
    for (int faceId = 0;
         faceId <
             hPoly.rows();
         ++faceId)
    {
        const Eigen::Vector3d normal =
            hPoly.block<1, 3>(
                    faceId,
                    0)
                .transpose();

        const double normalNorm =
            normal.norm();

        if (!std::isfinite(
                normalNorm) ||
            normalNorm <=
                epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }

        const auto support =
            protectedMincoPieceDirectionalSupport(
                piece,
                normal,
                overlapRadius,
                rootTolerance);

        if (!support.valid)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }

        const double protectedValue =
            support.support +
            hPoly(
                faceId,
                3);

        if (protectedValue >
            epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }
    }

    localDiagnostics.trajectory_contained =
        true;

    // The protected support explicitly contains the start and end
    // endpoint balls in every verified halfspace.
    localDiagnostics.endpoint_ball_guaranteed =
        true;

    // ------------------------------------------------------------
    // Authoritative safety scan against the COMPLETE obstacle cloud.
    //
    // This includes points outside the local candidate crop. Such
    // points must be excluded by a fixed domain/map halfspace.
    // ------------------------------------------------------------
    for (const Eigen::Vector3d &point :
         points)
    {
        const Eigen::Vector4d pointH(
            point(0),
            point(1),
            point(2),
            1.0);

        if ((hPoly *
             pointH)
                .maxCoeff() <=
            epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }
    }

    localDiagnostics.safety_verified =
        true;

    if (diagnostics != nullptr)
    {
        *diagnostics =
            localDiagnostics;
    }

    return true;
}

} // namespace traj_relevant

#endif
