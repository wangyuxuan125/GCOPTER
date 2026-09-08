#ifndef ACTIVE_SET_PROJECTION_HPP
#define ACTIVE_SET_PROJECTION_HPP

#include <Eigen/Eigen>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

struct ActiveSetProjectionResult
{
    bool success = false;

    // 0 = no failure / success
    // 1 = invalid input
    // 2 = nonfinite multiplier
    // 3 = nonfinite solution
    // 4 = nonfinite residual
    // 5 = active-row reselection
    // 6 = iteration cap
    int failure_reason = 0;

    int failure_constraint = -1;

    double failure_constraint_residual =
        std::numeric_limits<double>::infinity();

    int iterations = 0;

    int working_set_size = 0;

    double max_primal_violation =
        std::numeric_limits<double>::infinity();

    double min_active_multiplier =
        std::numeric_limits<double>::infinity();

    double equality_residual =
        std::numeric_limits<double>::infinity();

    Eigen::VectorXd solution;
};


// ================================================================
// Euclidean projection onto a finite intersection of halfspaces:
//
//     min_z  0.5 ||z-z0||^2
//
//     s.t.   A z <= b.
//
// This is a dense active-set solver intended for the small
// certificate-guided Bernstein cut set.
//
// At every iteration:
//
//   1. solve the equality-constrained projection for the current
//      working set;
//   2. remove an active inequality with a negative multiplier;
//   3. otherwise add the globally most violated inequality.
//
// When all active multipliers are nonnegative and every inactive
// inequality is satisfied, the KKT conditions of the strictly
// convex projection QP are satisfied.
// ================================================================
inline ActiveSetProjectionResult
solveEuclideanHalfspaceProjectionActiveSet(
    const Eigen::VectorXd &z0,
    const std::vector<Eigen::VectorXd> &rows,
    const std::vector<double> &rhs,
    const double primalTolerance,
    const double dualTolerance)
{
    ActiveSetProjectionResult result;

    const int constraintCount =
        static_cast<int>(rows.size());

    const int variableDimension =
        z0.size();

    if (!z0.allFinite() ||
        rows.size() != rhs.size() ||
        variableDimension <= 0 ||
        primalTolerance < 0.0 ||
        dualTolerance < 0.0)
    {
        result.failure_reason = 1;
        return result;
    }

    if (constraintCount == 0)
    {
        result.success = true;
        result.solution = z0;

        result.max_primal_violation = 0.0;
        result.min_active_multiplier = 0.0;
        result.equality_residual = 0.0;

        return result;
    }

    Eigen::MatrixXd A(
        constraintCount,
        variableDimension);

    Eigen::VectorXd b(
        constraintCount);

    for (int rowId = 0;
         rowId < constraintCount;
         ++rowId)
    {
        if (rows[rowId].size() !=
                variableDimension ||
            !rows[rowId].allFinite() ||
            !std::isfinite(rhs[rowId]))
        {
            result.failure_reason = 1;
            return result;
        }

        A.row(rowId) =
            rows[rowId].transpose();

        b(rowId) =
            rhs[rowId];
    }

    // The iteration cap is derived from problem size and is only a
    // cycling/numerical safeguard, not a planning hyperparameter.
    const int maxIterations =
        std::max(
            64,
            8 * constraintCount +
                4 * variableDimension);

    std::vector<int> workingSet;

    std::vector<bool> isActive(
        constraintCount,
        false);

    Eigen::VectorXd z =
        z0;

    for (int iteration = 0;
         iteration < maxIterations;
         ++iteration)
    {
        result.iterations =
            iteration + 1;

        Eigen::VectorXd multipliers;

        // ------------------------------------------------------------
        // Equality-constrained Euclidean projection:
        //
        //   min 1/2 ||z-z0||^2
        //   s.t. A_W z = b_W.
        //
        // KKT gives
        //
        //   z = z0 - A_W^T lambda
        //
        //   (A_W A_W^T) lambda =
        //       A_W z0 - b_W.
        //
        // CompleteOrthogonalDecomposition handles dependent rows.
        // ------------------------------------------------------------
        if (workingSet.empty())
        {
            z =
                z0;

            multipliers.resize(0);

            result.equality_residual =
                0.0;

            result.min_active_multiplier =
                0.0;
        }
        else
        {
            const int activeCount =
                static_cast<int>(
                    workingSet.size());

            Eigen::MatrixXd Aw(
                activeCount,
                variableDimension);

            Eigen::VectorXd bw(
                activeCount);

            for (int activeId = 0;
                 activeId < activeCount;
                 ++activeId)
            {
                Aw.row(activeId) =
                    A.row(
                        workingSet[activeId]);

                bw(activeId) =
                    b(
                        workingSet[activeId]);
            }

            const Eigen::MatrixXd gram =
                Aw * Aw.transpose();

            const Eigen::VectorXd c =
                Aw * z0 - bw;

            multipliers =
                gram
                    .completeOrthogonalDecomposition()
                    .solve(c);

            if (!multipliers.allFinite())
            {
                result.failure_reason = 2;
                return result;
            }

            z =
                z0 -
                Aw.transpose() *
                    multipliers;

            if (!z.allFinite())
            {
                result.failure_reason = 3;
                return result;
            }

            result.equality_residual =
                (Aw * z - bw)
                    .lpNorm<Eigen::Infinity>();

            result.min_active_multiplier =
                multipliers.minCoeff();

            // --------------------------------------------------------
            // Inequality KKT condition:
            //
            //     lambda >= 0.
            //
            // If one active multiplier is negative, that inequality
            // cannot belong to the optimal active set. Remove the
            // most negative one and resolve.
            // --------------------------------------------------------
            Eigen::Index minIndex = 0;

            const double minLambda =
                multipliers.minCoeff(
                    &minIndex);

            if (minLambda <
                -dualTolerance)
            {
                const int constraintId =
                    workingSet[
                        static_cast<int>(
                            minIndex)];

                isActive[constraintId] =
                    false;

                workingSet.erase(
                    workingSet.begin() +
                    static_cast<int>(
                        minIndex));

                continue;
            }
        }

        // ------------------------------------------------------------
        // Search globally for the most violated inequality.
        // ------------------------------------------------------------
        const Eigen::VectorXd residual =
            A * z - b;

        if (!residual.allFinite())
        {
            result.failure_reason = 4;
            return result;
        }

        Eigen::Index worstIndex = 0;

        const double maxViolation =
            residual.maxCoeff(
                &worstIndex);

        result.max_primal_violation =
            std::max(
                0.0,
                maxViolation);

        result.working_set_size =
            static_cast<int>(
                workingSet.size());

        // All primal constraints satisfied and all active
        // multipliers nonnegative -> KKT solution.
        if (maxViolation <=
            primalTolerance)
        {
            result.success =
                true;

            result.solution =
                z;

            return result;
        }

        const int violatedId =
            static_cast<int>(
                worstIndex);

        // A currently active row should satisfy its equality up to
        // numerical tolerance. Re-selecting it means the equality
        // solve is numerically inconsistent.
        if (isActive[violatedId])
        {
            result.failure_reason = 5;
            result.failure_constraint =
                violatedId;
            result.failure_constraint_residual =
                maxViolation;
            result.solution =
                z;
            return result;
        }

        isActive[violatedId] =
            true;

        workingSet.push_back(
            violatedId);
    }

    result.failure_reason = 6;
    result.solution = z;

    return result;
}

} // namespace traj_relevant

#endif