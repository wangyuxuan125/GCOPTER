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
