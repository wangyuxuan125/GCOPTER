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
