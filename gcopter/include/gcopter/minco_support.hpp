#ifndef MINCO_SUPPORT_HPP
#define MINCO_SUPPORT_HPP

#include "trajectory.hpp"
#include "root_finder.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace traj_relevant
{

struct DirectionalSupportResult
{
    bool valid =
        false;

    double support =
        -std::numeric_limits<double>::infinity();

    // tau in [0, 1].
    double normalized_time =
        0.0;

    double physical_time =
        0.0;

    int stationary_point_count =
        0;
};

struct MetricClosestPointResult
{
    bool valid =
        false;

    double normalized_time =
        0.0;

    double physical_time =
        0.0;

    Eigen::Vector3d point =
        Eigen::Vector3d::Zero();

    double metric_distance_squared =
        std::numeric_limits<double>::infinity();

    double euclidean_distance =
        std::numeric_limits<double>::infinity();

    int stationary_point_count =
        0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ================================================================
// Continuous-time directional support of one polynomial piece:
//
//     h_Gamma(n)
//       = max_{t in [0,T]} n^T p(t).
//
// For a degree-D polynomial,
// stationary points satisfy
//
//     n^T p'(t) = 0,
//
// which is degree D-1.
//
// For GCOPTER's quintic MINCO piece (D=5), this is only quartic.
//
// The calculation is performed in normalized time:
//
//     tau = t / T in [0,1],
//
// improving numerical conditioning.
//
// Piece<D>::normalizePosCoeffMat() already converts coefficients
// from physical time t to normalized time tau.
// ================================================================
template <int D>
inline DirectionalSupportResult
exactMincoDirectionalSupport(
    const Piece<D> &piece,
    const Eigen::Vector3d &normal,
    const double rootTolerance =
        1.0e-10,
    const double coefficientTolerance =
        1.0e-12)
{
    static_assert(
        D >= 1,
        "Directional support requires degree >= 1.");

    DirectionalSupportResult result;

    if (!normal.allFinite())
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

    const double duration =
        piece.getDuration();

    if (!std::isfinite(duration) ||
        duration <= 0.0)
    {
        return result;
    }

    // ------------------------------------------------------------
    // normalizePosCoeffMat() preserves GCOPTER's coefficient order:
    //
    //     col(0) = tau^D coefficient
    //     ...
    //     col(D) = constant coefficient.
    //
    // Therefore scalarCoeffs is already in the descending-order
    // convention expected by RootFinder.
    // ------------------------------------------------------------
    const auto normalizedPositionCoeffs =
        piece.normalizePosCoeffMat();

    Eigen::VectorXd scalarCoeffs(
        D + 1);

    for (int coefficientId = 0;
         coefficientId <= D;
         ++coefficientId)
    {
        scalarCoeffs(
            coefficientId) =
            normal.dot(
                normalizedPositionCoeffs.col(
                    coefficientId));
    }

    // ------------------------------------------------------------
    // Derivative coefficients:
    //
    // c_D tau^D + ... + c_1 tau + c_0
    //
    // ->
    //
    // D c_D tau^(D-1) + ... + c_1.
    // ------------------------------------------------------------
    Eigen::VectorXd derivativeCoeffs(
        D);

    for (int coefficientId = 0;
         coefficientId < D;
         ++coefficientId)
    {
        derivativeCoeffs(
            coefficientId) =
            static_cast<double>(
                D - coefficientId) *
            scalarCoeffs(
                coefficientId);
    }

    auto evaluateAtNormalizedTime =
        [&](double tau)
        {
            tau =
                std::max(
                    0.0,
                    std::min(
                        1.0,
                        tau));

            return normal.dot(
                piece.getPos(
                    tau *
                    duration));
        };

    // Endpoints always belong to the candidate set.
    const double valueAtStart =
        evaluateAtNormalizedTime(
            0.0);

    const double valueAtEnd =
        evaluateAtNormalizedTime(
            1.0);

    result.support =
        valueAtStart;

    result.normalized_time =
        0.0;

    if (valueAtEnd >
        result.support)
    {
        result.support =
            valueAtEnd;

        result.normalized_time =
            1.0;
    }

    // ------------------------------------------------------------
    // Scale the derivative polynomial before root solving.
    // Multiplication by a positive scalar does not change its roots
    // and improves coefficient conditioning.
    // ------------------------------------------------------------
    const double derivativeScale =
        derivativeCoeffs
            .cwiseAbs()
            .maxCoeff();

    if (std::isfinite(
            derivativeScale) &&
        derivativeScale >
            coefficientTolerance)
    {
        const Eigen::VectorXd scaledDerivative =
            derivativeCoeffs /
            derivativeScale;

        const std::set<double>
            stationaryPoints =
                RootFinder::
                    solvePolynomial(
                        scaledDerivative,
                        0.0,
                        1.0,
                        rootTolerance);

        result.stationary_point_count =
            static_cast<int>(
                stationaryPoints.size());

        for (const double tau :
             stationaryPoints)
        {
            if (!std::isfinite(tau) ||
                tau <= 0.0 ||
                tau >= 1.0)
            {
                continue;
            }

            const double value =
                evaluateAtNormalizedTime(
                    tau);

            if (value >
                result.support)
            {
                result.support =
                    value;

                result.normalized_time =
                    tau;
            }
        }
    }

    result.physical_time =
        result.normalized_time *
        duration;

    result.valid =
        std::isfinite(
            result.support);

    return result;
}

struct ProtectedDirectionalSupportResult
{
    bool valid =
        false;

    double support =
        -std::numeric_limits<double>::infinity();

    Eigen::Vector3d point =
        Eigen::Vector3d::Zero();

    // 0: curve
    // 1: start endpoint ball
    // 2: end endpoint ball
    int source =
        -1;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ================================================================
// Exact support mapping of
//
//   K = conv(
//         Gamma
//         union B(p(0), r)
//         union B(p(T), r)).
//
// Only the endpoint balls are protected for corridor overlap.
// The complete trajectory itself is contained exactly, but is NOT
// unnecessarily thickened by the overlap radius.
// ================================================================
template <int D>
inline ProtectedDirectionalSupportResult
protectedMincoPieceDirectionalSupport(
    const Piece<D> &piece,
    const Eigen::Vector3d &normal,
    const double endpointRadius,
    const double rootTolerance =
        1.0e-10,
    const double coefficientTolerance =
        1.0e-12)
{
    ProtectedDirectionalSupportResult result;

    if (!normal.allFinite())
    {
        return result;
    }

    const double normalNorm =
        normal.norm();

    if (!std::isfinite(normalNorm) ||
        normalNorm <=
            coefficientTolerance)
    {
        return result;
    }

    const auto curveSupport =
        exactMincoDirectionalSupport(
            piece,
            normal,
            rootTolerance,
            coefficientTolerance);

    if (!curveSupport.valid)
    {
        return result;
    }

    const double duration =
        piece.getDuration();

    const Eigen::Vector3d startPoint =
        piece.getPos(
            0.0);

    const Eigen::Vector3d endPoint =
        piece.getPos(
            duration);

    const double radius =
        std::max(
            0.0,
            endpointRadius);

    const Eigen::Vector3d unitNormal =
        normal /
        normalNorm;

    // Curve support.
    result.valid =
        true;

    result.support =
        curveSupport.support;

    result.point =
        piece.getPos(
            curveSupport.physical_time);

    result.source =
        0;

    // Start junction ball.
    const double startBallSupport =
        normal.dot(
            startPoint) +
        radius *
            normalNorm;

    if (startBallSupport >
        result.support)
    {
        result.support =
            startBallSupport;

        result.point =
            startPoint +
            radius *
                unitNormal;

        result.source =
            1;
    }

    // End junction ball.
    const double endBallSupport =
        normal.dot(
            endPoint) +
        radius *
            normalNorm;

    if (endBallSupport >
        result.support)
    {
        result.support =
            endBallSupport;

        result.point =
            endPoint +
            radius *
                unitNormal;

        result.source =
            2;
    }

    return result;
}

struct MincoPolytopeContainmentCertificate
{
    bool valid =
        false;

    bool contained =
        false;

    int checked_face_count =
        0;

    int worst_face =
        -1;

    // Signed Euclidean violation:
    //
    // > 0 : trajectory leaves the halfspace
    // = 0 : touches the face
    // < 0 : inside
    double max_signed_violation_m =
        -std::numeric_limits<double>::infinity();

    // Positive when contained.
    double min_margin_m =
        std::numeric_limits<double>::infinity();

    double worst_normalized_time =
        0.0;

    double worst_physical_time =
        0.0;
};

// ================================================================
// Exact continuous-time certificate:
//
//   Gamma = {p(t) : 0 <= t <= T}
//
//   P = intersection_f {x : n_f^T x + d_f <= 0}
//
// For each face:
//
//   max_t [n_f^T p(t) + d_f]
//      = h_Gamma(n_f) + d_f.
//
// Therefore:
//
//   Gamma subset P
//
// iff every face has non-positive maximum violation.
//
// The returned violation is divided by ||n_f|| so that it is in
// meters even if input halfspaces are not normalized.
// ================================================================
template <int D>
inline MincoPolytopeContainmentCertificate
certifyMincoPieceInPolytope(
    const Piece<D> &piece,
    const Eigen::MatrixX4d &hPoly,
    const double containmentToleranceM =
        1.0e-6,
    const double rootTolerance =
        1.0e-10,
    const double coefficientTolerance =
        1.0e-12)
{
    MincoPolytopeContainmentCertificate result;

    if (hPoly.rows() <= 0 ||
        hPoly.cols() != 4 ||
        !hPoly.allFinite())
    {
        return result;
    }

    for (int faceId = 0;
         faceId < hPoly.rows();
         ++faceId)
    {
        const Eigen::Vector3d normal =
            hPoly.block<1, 3>(
                    faceId,
                    0)
                .transpose();

        const double normalNorm =
            normal.norm();

        if (!normal.allFinite() ||
            !std::isfinite(normalNorm) ||
            normalNorm <=
                coefficientTolerance)
        {
            return result;
        }

        const auto support =
            exactMincoDirectionalSupport(
                piece,
                normal,
                rootTolerance,
                coefficientTolerance);

        if (!support.valid)
        {
            return result;
        }

        const double rawViolation =
            support.support +
            hPoly(
                faceId,
                3);

        const double signedViolationM =
            rawViolation /
            normalNorm;

        ++result.checked_face_count;

        if (signedViolationM >
            result.max_signed_violation_m)
        {
            result.max_signed_violation_m =
                signedViolationM;

            result.worst_face =
                faceId;

            result.worst_normalized_time =
                support.normalized_time;

            result.worst_physical_time =
                support.physical_time;
        }

        result.min_margin_m =
            std::min(
                result.min_margin_m,
                -signedViolationM);
    }

    result.valid =
        result.checked_face_count ==
        hPoly.rows();

    result.contained =
        result.valid &&
        result.max_signed_violation_m <=
            containmentToleranceM;

    return result;
}

// ================================================================
// Exact metric closest point on one MINCO polynomial piece.
//
// Solve
//
//   min_{tau in [0,1]}
//       (query - p(tau))^T W (query - p(tau)).
//
// For quintic MINCO:
//
//   p       : degree 5
//   p'      : degree 4
//
// stationary equation
//
//   p'(tau)^T W (p(tau) - query) = 0
//
// therefore has degree at most 9.
//
// All real roots in (0,1), together with both endpoints, are
// evaluated explicitly.
// ================================================================
template <int D>
inline MetricClosestPointResult
exactMincoMetricClosestPoint(
    const Piece<D> &piece,
    const Eigen::Vector3d &query,
    const Eigen::Matrix3d &metric,
    const double rootTolerance =
        1.0e-10,
    const double coefficientTolerance =
        1.0e-12)
{
    static_assert(
        D >= 1,
        "Closest point requires polynomial degree >= 1.");

    MetricClosestPointResult result;

    if (!query.allFinite() ||
        !metric.allFinite())
    {
        return result;
    }

    const double duration =
        piece.getDuration();

    if (!std::isfinite(duration) ||
        duration <= 0.0)
    {
        return result;
    }

    // Symmetrize to suppress tiny numerical asymmetry.
    const Eigen::Matrix3d symmetricMetric =
        0.5 *
        (metric +
         metric.transpose());

    Eigen::SelfAdjointEigenSolver<
        Eigen::Matrix3d>
        metricSolver(
            symmetricMetric);

    if (metricSolver.info() !=
            Eigen::Success ||
        metricSolver.eigenvalues()
                .minCoeff() <=
            coefficientTolerance)
    {
        return result;
    }

    // ------------------------------------------------------------
    // Position polynomial in normalized time tau.
    //
    // Storage convention:
    //
    //   col(0) = tau^D
    //   ...
    //   col(D) = constant.
    //
    // This is also the descending coefficient order expected by
    // RootFinder.
    // ------------------------------------------------------------
    const auto positionCoeffs =
        piece.normalizePosCoeffMat();

    Eigen::Matrix<double, 3, D + 1>
        residualCoeffs =
            positionCoeffs;

    // Only the constant coefficient changes for p(tau)-query.
    residualCoeffs.col(D) -=
        query;

    // p'(tau), also in descending power order.
    Eigen::Matrix<double, 3, D>
        velocityCoeffs;

    for (int coefficientId = 0;
         coefficientId < D;
         ++coefficientId)
    {
        velocityCoeffs.col(
            coefficientId) =
            static_cast<double>(
                D - coefficientId) *
            positionCoeffs.col(
                coefficientId);
    }

    // ------------------------------------------------------------
    // Construct
    //
    //   g(tau) =
    //       p'(tau)^T W (p(tau)-query).
    //
    // Degree:
    //
    //   (D-1) + D = 2D-1.
    //
    // Hence number of coefficients = 2D.
    // ------------------------------------------------------------
    Eigen::VectorXd stationaryCoeffs(
        2 * D);

    stationaryCoeffs.setZero();

    for (int velocityId = 0;
         velocityId < D;
         ++velocityId)
    {
        for (int residualId = 0;
             residualId <= D;
             ++residualId)
        {
            stationaryCoeffs(
                velocityId +
                residualId) +=
                velocityCoeffs
                    .col(
                        velocityId)
                    .dot(
                        symmetricMetric *
                        residualCoeffs.col(
                            residualId));
        }
    }

    auto evaluateCandidate =
        [&](const double rawTau)
        {
            const double tau =
                std::max(
                    0.0,
                    std::min(
                        1.0,
                        rawTau));

            const Eigen::Vector3d point =
                piece.getPos(
                    tau *
                    duration);

            const Eigen::Vector3d delta =
                query -
                point;

            const double metricDistanceSquared =
                delta.dot(
                    symmetricMetric *
                    delta);

            if (!std::isfinite(
                    metricDistanceSquared))
            {
                return;
            }

            if (metricDistanceSquared <
                result.metric_distance_squared)
            {
                result.metric_distance_squared =
                    std::max(
                        0.0,
                        metricDistanceSquared);

                result.normalized_time =
                    tau;

                result.point =
                    point;

                result.euclidean_distance =
                    delta.norm();
            }
        };

    // Endpoints must always be considered.
    evaluateCandidate(
        0.0);

    evaluateCandidate(
        1.0);

    // ------------------------------------------------------------
    // Polynomial scaling does not change roots.
    // ------------------------------------------------------------
    const double coefficientScale =
        stationaryCoeffs
            .cwiseAbs()
            .maxCoeff();

    if (std::isfinite(
            coefficientScale) &&
        coefficientScale >
            coefficientTolerance)
    {
        const Eigen::VectorXd scaledCoeffs =
            stationaryCoeffs /
            coefficientScale;

        // RootFinder's Sturm isolation requires nonzero boundary
        // evaluations. Endpoints are already handled explicitly,
        // therefore search only an open numerical interior.
        const double boundaryOffset =
            std::max(
                1.0e-12,
                10.0 *
                    rootTolerance);

        const double lowerBound =
            boundaryOffset;

        const double upperBound =
            1.0 -
            boundaryOffset;

        if (lowerBound <
            upperBound)
        {
            const std::set<double>
                roots =
                    RootFinder::
                        solvePolynomial(
                            scaledCoeffs,
                            lowerBound,
                            upperBound,
                            rootTolerance);

            result.stationary_point_count =
                static_cast<int>(
                    roots.size());

            for (const double tau :
                 roots)
            {
                if (!std::isfinite(tau))
                {
                    continue;
                }

                evaluateCandidate(
                    tau);
            }
        }
    }

    result.physical_time =
        result.normalized_time *
        duration;

    result.valid =
        std::isfinite(
            result.metric_distance_squared) &&
        result.point.allFinite();

    return result;
}

struct ProtectedSetProjectionResult
{
    bool valid =
        false;

    bool converged =
        false;

    bool certified_separable =
        false;

    Eigen::Vector3d point =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d separating_normal =
        Eigen::Vector3d::Zero();

    double metric_distance_squared =
        std::numeric_limits<double>::infinity();

    // Frank-Wolfe lower bound on the true squared metric distance.
    double metric_distance_squared_lower_bound =
        0.0;

    double euclidean_distance =
        std::numeric_limits<double>::infinity();

    // Exact support-certified Euclidean separation margin.
    //
    // > 0 means a valid separating halfspace definitely exists.
    double certified_separation_margin_m =
        -std::numeric_limits<double>::infinity();

    double frank_wolfe_gap =
        std::numeric_limits<double>::infinity();

    int iterations =
        0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ================================================================
// Project query point o onto the convex protected MINCO set
//
//   K = conv(
//         Gamma
//         union B(p(0), r)
//         union B(p(T), r))
//
// in metric W:
//
//   min_{q in K}
//       1/2 (q-o)^T W (q-o).
//
// Frank-Wolfe linear minimization uses the exact support mapping:
//
//   s_k
//     = argmin_{x in K} grad^T x
//     = supportPoint_K(-grad).
//
// After projection, candidate normal
//
//   n = W(o-q)
//
// is checked AGAIN using exact support. Therefore
// certified_separable=true is a genuine separation certificate,
// independent of Frank-Wolfe approximation error.
// ================================================================
template <int D>
inline ProtectedSetProjectionResult
projectOntoProtectedMincoSet(
    const Piece<D> &piece,
    const Eigen::Vector3d &query,
    const Eigen::Matrix3d &metric,
    const double endpointRadius,
    const int maxIterations =
        128,
    const double gapTolerance =
        1.0e-10,
    const double distanceTolerance =
        1.0e-10,
    const double separationToleranceM =
        1.0e-6,
    const double rootTolerance =
        1.0e-10,
    const double coefficientTolerance =
        1.0e-12)
{
    ProtectedSetProjectionResult result;

    if (!query.allFinite() ||
        !metric.allFinite() ||
        maxIterations <= 0)
    {
        return result;
    }

    const Eigen::Matrix3d symmetricMetric =
        0.5 *
        (metric +
         metric.transpose());

    Eigen::SelfAdjointEigenSolver<
        Eigen::Matrix3d>
        metricSolver(
            symmetricMetric);

    if (metricSolver.info() !=
            Eigen::Success ||
        metricSolver.eigenvalues()
                .minCoeff() <=
            coefficientTolerance)
    {
        return result;
    }

    // A point on Gamma is already a valid point in K.
    // The exact curve projection is therefore a useful warm start.
    const auto curveClosest =
        exactMincoMetricClosestPoint(
            piece,
            query,
            symmetricMetric,
            rootTolerance,
            coefficientTolerance);

    if (!curveClosest.valid)
    {
        return result;
    }

    Eigen::Vector3d q =
        curveClosest.point;

    double finalGap =
        std::numeric_limits<double>::
            infinity();

    for (int iteration = 0;
         iteration < maxIterations;
         ++iteration)
    {
        const Eigen::Vector3d delta =
            q -
            query;

        const Eigen::Vector3d gradient =
            symmetricMetric *
            delta;

        const double metricDistanceSquared =
            delta.dot(
                gradient);

        if (!std::isfinite(
                metricDistanceSquared))
        {
            return result;
        }

        if (metricDistanceSquared <=
            distanceTolerance *
            distanceTolerance)
        {
            finalGap =
                0.0;

            result.converged =
                true;

            result.iterations =
                iteration;

            break;
        }

        // Linear minimization oracle:
        //
        // min grad^T x
        // =
        // max (-grad)^T x.
        const auto linearOracle =
            protectedMincoPieceDirectionalSupport(
                piece,
                -gradient,
                endpointRadius,
                rootTolerance,
                coefficientTolerance);

        if (!linearOracle.valid ||
            !linearOracle.point.allFinite())
        {
            return result;
        }

        const Eigen::Vector3d direction =
            linearOracle.point -
            q;

        double gap =
            gradient.dot(
                q -
                linearOracle.point);

        // The exact FW gap should be nonnegative.
        if (gap < 0.0 &&
            gap >
                -1.0e-12)
        {
            gap =
                0.0;
        }

        if (!std::isfinite(gap) ||
            gap < 0.0)
        {
            return result;
        }

        finalGap =
            gap;

        result.iterations =
            iteration + 1;

        const double gapScale =
            std::max(
                1.0,
                metricDistanceSquared);

        if (gap <=
            gapTolerance *
                gapScale)
        {
            result.converged =
                true;

            break;
        }

        const double denominator =
            direction.dot(
                symmetricMetric *
                direction);

        if (!std::isfinite(
                denominator) ||
            denominator <=
                coefficientTolerance)
        {
            // If the linear oracle cannot provide a meaningful
            // direction, accept only if the gap is already tiny.
            if (gap <=
                gapTolerance *
                    gapScale)
            {
                result.converged =
                    true;

                break;
            }

            return result;
        }

        // Exact line search for the quadratic objective.
        double alpha =
            gap /
            denominator;

        alpha =
            std::max(
                0.0,
                std::min(
                    1.0,
                    alpha));

        q +=
            alpha *
            direction;
    }

    // Recompute the final Frank-Wolfe gap.
    const Eigen::Vector3d finalDelta =
        q -
        query;

    const Eigen::Vector3d finalGradient =
        symmetricMetric *
        finalDelta;

    const double metricDistanceSquared =
        finalDelta.dot(
            finalGradient);

    if (!std::isfinite(
            metricDistanceSquared))
    {
        return result;
    }

    const auto finalLinearOracle =
        protectedMincoPieceDirectionalSupport(
            piece,
            -finalGradient,
            endpointRadius,
            rootTolerance,
            coefficientTolerance);

    if (finalLinearOracle.valid &&
        finalLinearOracle.point.allFinite())
    {
        finalGap =
            finalGradient.dot(
                q -
                finalLinearOracle.point);

        if (finalGap < 0.0 &&
            finalGap >
                -1.0e-12)
        {
            finalGap =
                0.0;
        }
    }

    if (!std::isfinite(finalGap) ||
        finalGap < 0.0)
    {
        return result;
    }

    result.point =
        q;

    result.metric_distance_squared =
        std::max(
            0.0,
            metricDistanceSquared);

    result.euclidean_distance =
        finalDelta.norm();

    result.frank_wolfe_gap =
        finalGap;

    // For f(q) = 1/2 d^2, the FW gap gives
    //
    //   f(q) - f* <= gap
    //
    // hence
    //
    //   d*^2 >= d(q)^2 - 2 gap.
    result.metric_distance_squared_lower_bound =
        std::max(
            0.0,
            result.metric_distance_squared -
                2.0 *
                    finalGap);

    const double finalGapScale =
        std::max(
            1.0,
            result.metric_distance_squared);

    if (finalGap <=
        gapTolerance *
            finalGapScale)
    {
        result.converged =
            true;
    }

    // ------------------------------------------------------------
    // Exact separation certificate.
    //
    // For the exact metric projection:
    //
    //   n = W(o-q*)
    //
    // satisfies
    //
    //   n^T x <= n^T q*,  for all x in K.
    //
    // Even with an approximate q, we do NOT assume this relation.
    // Instead, exact support is queried again.
    // ------------------------------------------------------------
    const Eigen::Vector3d normal =
        symmetricMetric *
        (query -
         q);

    const double normalNorm =
        normal.norm();

    if (normalNorm >
            coefficientTolerance &&
        normal.allFinite())
    {
        const auto protectedSupport =
            protectedMincoPieceDirectionalSupport(
                piece,
                normal,
                endpointRadius,
                rootTolerance,
                coefficientTolerance);

        if (!protectedSupport.valid)
        {
            return result;
        }

        const double rawSeparation =
            normal.dot(
                query) -
            protectedSupport.support;

        result.separating_normal =
            normal /
            normalNorm;

        result.certified_separation_margin_m =
            rawSeparation /
            normalNorm;

        result.certified_separable =
            result.certified_separation_margin_m >
            separationToleranceM;
    }
    else
    {
        // Query is numerically on/in K.
        result.certified_separation_margin_m =
            0.0;

        result.certified_separable =
            false;
    }

    result.valid =
        result.point.allFinite() &&
        std::isfinite(
            result.metric_distance_squared) &&
        std::isfinite(
            result.frank_wolfe_gap) &&
        std::isfinite(
            result.certified_separation_margin_m);

    return result;
}

template <int D>
inline double
denseMetricClosestPointUpperBound(
    const Piece<D> &piece,
    const Eigen::Vector3d &query,
    const Eigen::Matrix3d &metric,
    const int sampleCount =
        4001)
{
    if (!query.allFinite() ||
        !metric.allFinite() ||
        sampleCount < 2)
    {
        return
            std::numeric_limits<double>::
                quiet_NaN();
    }

    const Eigen::Matrix3d symmetricMetric =
        0.5 *
        (metric +
         metric.transpose());

    const double duration =
        piece.getDuration();

    double minimumDistanceSquared =
        std::numeric_limits<double>::
            infinity();

    for (int sampleId = 0;
         sampleId < sampleCount;
         ++sampleId)
    {
        const double tau =
            static_cast<double>(
                sampleId) /
            static_cast<double>(
                sampleCount - 1);

        const Eigen::Vector3d delta =
            query -
            piece.getPos(
                tau *
                duration);

        const double distanceSquared =
            delta.dot(
                symmetricMetric *
                delta);

        minimumDistanceSquared =
            std::min(
                minimumDistanceSquared,
                distanceSquared);
    }

    return minimumDistanceSquared;
}

// ================================================================
// Bernstein convex-hull upper bound.
//
// This is NOT used by the final algorithm.
// It is retained as an independent validation bound:
//
//     h_exact <= h_Bernstein.
//
// For
//
//     p(tau) = sum_{k=0}^D a_k tau^k,
//
// Bernstein control point i is
//
//     b_i = sum_{k=0}^i
//             C(i,k) / C(D,k) * a_k.
// ================================================================
inline double binomialCoefficient(
    const int n,
    const int k)
{
    if (k < 0 ||
        k > n)
    {
        return 0.0;
    }

    if (k == 0 ||
        k == n)
    {
        return 1.0;
    }

    const int kk =
        std::min(
            k,
            n - k);

    double value =
        1.0;

    for (int i = 1;
         i <= kk;
         ++i)
    {
        value *=
            static_cast<double>(
                n - kk + i);

        value /=
            static_cast<double>(
                i);
    }

    return value;
}

template <int D>
inline double
bernsteinDirectionalSupportUpperBound(
    const Piece<D> &piece,
    const Eigen::Vector3d &normal)
{
    if (!normal.allFinite())
    {
        return
            std::numeric_limits<double>::
                quiet_NaN();
    }

    const auto normalizedPositionCoeffs =
        piece.normalizePosCoeffMat();

    double support =
        -std::numeric_limits<double>::
            infinity();

    for (int bernsteinId = 0;
         bernsteinId <= D;
         ++bernsteinId)
    {
        Eigen::Vector3d controlPoint =
            Eigen::Vector3d::Zero();

        for (int powerId = 0;
             powerId <= bernsteinId;
             ++powerId)
        {
            // a_k is stored in column D-k.
            const Eigen::Vector3d powerCoefficient =
                normalizedPositionCoeffs.col(
                    D - powerId);

            const double weight =
                binomialCoefficient(
                    bernsteinId,
                    powerId) /
                binomialCoefficient(
                    D,
                    powerId);

            controlPoint +=
                weight *
                powerCoefficient;
        }

        support =
            std::max(
                support,
                normal.dot(
                    controlPoint));
    }

    return support;
}

// ================================================================
// Dense sampling lower bound used ONLY for validation:
//
//     h_sample <= h_exact.
//
// It is deliberately not part of the corridor safety certificate.
// ================================================================
template <int D>
inline double
denseDirectionalSupportLowerBound(
    const Piece<D> &piece,
    const Eigen::Vector3d &normal,
    const int sampleCount =
        2001)
{
    if (!normal.allFinite() ||
        sampleCount < 2)
    {
        return
            std::numeric_limits<double>::
                quiet_NaN();
    }

    const double duration =
        piece.getDuration();

    double support =
        -std::numeric_limits<double>::
            infinity();

    for (int sampleId = 0;
         sampleId < sampleCount;
         ++sampleId)
    {
        const double tau =
            static_cast<double>(
                sampleId) /
            static_cast<double>(
                sampleCount - 1);

        support =
            std::max(
                support,
                normal.dot(
                    piece.getPos(
                        tau *
                        duration)));
    }

    return support;
}

} // namespace traj_relevant

#endif
