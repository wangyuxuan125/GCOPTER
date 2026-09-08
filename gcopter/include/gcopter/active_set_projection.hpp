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

    // Direct KKT solve diagnostics.
    double stationarity_residual =
        std::numeric_limits<double>::infinity();

    double kkt_residual =
        std::numeric_limits<double>::infinity();

    int last_kkt_rank = 0;

    // Rank-revealing working-set diagnostics.
    int rank_compression_count = 0;

    int dropped_dependent_constraints = 0;

    int last_working_set_rank = 0;

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

            // --------------------------------------------------------
            // Rank-revealing compression of the active equality set.
            //
            // A valid Euclidean-projection optimum only requires a
            // linearly independent basis of binding inequalities.
            //
            // If Aw is row-rank deficient, forcing every dependent
            // row to equality can create an inconsistent least-squares
            // system even though the original inequality QP remains
            // feasible.
            //
            // Keep a pivoted independent basis and return all dependent
            // rows to the inactive inequality pool. If any dropped row
            // is genuinely needed, the global violation search can
            // activate it again later.
            // --------------------------------------------------------
            Eigen::ColPivHouseholderQR<
                Eigen::MatrixXd>
                    rankQr(
                        Aw.transpose());

            const int activeRank =
                rankQr.rank();

            result.last_working_set_rank =
                activeRank;

            if (activeRank <
                activeCount)
            {
                const auto permutation =
                    rankQr
                        .colsPermutation()
                        .indices();

                std::vector<bool> keep(
                    activeCount,
                    false);

                for (int basisId = 0;
                     basisId < activeRank;
                     ++basisId)
                {
                    const int activePosition =
                        permutation(
                            basisId);

                    if (activePosition < 0 ||
                        activePosition >=
                            activeCount)
                    {
                        result.failure_reason =
                            1;

                        return result;
                    }

                    keep[
                        activePosition] =
                            true;
                }

                std::vector<int>
                    reducedWorkingSet;

                reducedWorkingSet.reserve(
                    activeRank);

                for (int activeId = 0;
                     activeId <
                         activeCount;
                     ++activeId)
                {
                    const int constraintId =
                        workingSet[
                            activeId];

                    if (keep[
                            activeId])
                    {
                        reducedWorkingSet
                            .push_back(
                                constraintId);
                    }
                    else
                    {
                        isActive[
                            constraintId] =
                                false;
                    }
                }

                ++result
                      .rank_compression_count;

                result
                    .dropped_dependent_constraints +=
                        activeCount -
                        activeRank;

                workingSet.swap(
                    reducedWorkingSet);

                // Rebuild Aw/bw and solve the equality projection
                // using the compressed independent basis.
                continue;
            }

            // --------------------------------------------------------
            // Direct KKT solve.
            //
            // Avoid the normal equations
            //
            //     (Aw Aw^T) lambda = Aw z0 - bw
            //
            // because forming Aw Aw^T squares the condition number.
            //
            // Solve instead
            //
            //   [ I   Aw^T ] [ z      ] = [ z0 ]
            //   [ Aw   0   ] [ lambda ]   [ bw ].
            //
            // ColPivHouseholderQR is used directly on the KKT matrix
            // so the near-dependent geometry is not squared through
            // a Gram matrix.
            // --------------------------------------------------------
            const int kktDimension =
                variableDimension +
                activeCount;

            Eigen::MatrixXd kkt =
                Eigen::MatrixXd::Zero(
                    kktDimension,
                    kktDimension);

            kkt.topLeftCorner(
                    variableDimension,
                    variableDimension)
                .setIdentity();

            kkt.topRightCorner(
                    variableDimension,
                    activeCount) =
                Aw.transpose();

            kkt.bottomLeftCorner(
                    activeCount,
                    variableDimension) =
                Aw;

            Eigen::VectorXd kktRhs(
                kktDimension);

            kktRhs.head(
                variableDimension) =
                z0;

            kktRhs.tail(
                activeCount) =
                bw;

            Eigen::ColPivHouseholderQR<
                Eigen::MatrixXd>
                    kktQr(kkt);

            result.last_kkt_rank =
                kktQr.rank();

            const Eigen::VectorXd
                kktSolution =
                    kktQr.solve(
                        kktRhs);

            if (!kktSolution.allFinite())
            {
                result.failure_reason = 3;
                return result;
            }

            z =
                kktSolution.head(
                    variableDimension);

            multipliers =
                kktSolution.tail(
                    activeCount);

            if (!z.allFinite())
            {
                result.failure_reason = 3;
                return result;
            }

            if (!multipliers.allFinite())
            {
                result.failure_reason = 2;
                return result;
            }

            result.equality_residual =
                (Aw * z - bw)
                    .lpNorm<Eigen::Infinity>();

            result.stationarity_residual =
                (
                    z -
                    z0 +
                    Aw.transpose() *
                        multipliers
                ).lpNorm<Eigen::Infinity>();

            result.kkt_residual =
                (
                    kkt *
                        kktSolution -
                    kktRhs
                ).lpNorm<Eigen::Infinity>();

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