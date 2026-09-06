#pragma once

#include "gcopter/sdlp.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>


namespace gcopter_benchmark
{

struct EffectiveFaceMetric
{
    bool valid = false;

    int raw_rows = 0;

    // Number of distinct normalized geometric H-planes
    // before redundancy testing.
    int unique_plane_groups = 0;

    // raw_rows - unique_plane_groups.
    int duplicate_rows = 0;

    // Distinct geometric planes whose removal enlarges
    // the represented polytope.
    int effective_faces = 0;

    // Distinct geometric planes implied by the others.
    int redundant_plane_groups = 0;

    // If removal of one effective face makes the support
    // LP unbounded, that face is certainly effective.
    int unbounded_support_tests = 0;

    // Maximum finite support violation among redundant
    // groups.  Should remain <= activity tolerance.
    double max_redundant_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();

    // Minimum finite positive support violation among
    // effective groups.
    double min_finite_effective_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();
};


// ============================================================
// Common effective-face evaluator.
//
// Input convention:
//
//     n^T x + d <= 0
//
// Each row is first normalized by ||n||, making all support
// violations metric distances in metres.
//
// Duplicate planes are grouped BEFORE LP testing.  This avoids
// the classic error where two identical rows each make the
// other appear redundant.
//
// For each unique plane group G:
//
//     maximize n_G^T x + d_G
//
// subject to every row NOT in G.
//
// Using sdlp:
//
//     minimize (-n_G)^T x.
//
// If the optimum support violation is > activityToleranceM,
// removing this plane changes the polytope, hence the plane is
// effective.
//
// If the support problem becomes unbounded, the removed plane
// is also necessarily effective.
//
// This function MEASURES redundancy only.  It never modifies
// the original H-polytope.
// ============================================================
inline EffectiveFaceMetric
evaluateEffectiveFaces(
    const Eigen::MatrixX4d &hPoly,
    const double duplicateNormalTolerance =
        1.0e-9,
    const double duplicateOffsetToleranceM =
        1.0e-9,
    const double activityToleranceM =
        1.0e-8,
    const double normalEpsilon =
        1.0e-12)
{
    EffectiveFaceMetric result;

    const int rowCount =
        static_cast<int>(
            hPoly.rows());

    result.raw_rows =
        rowCount;


    if (rowCount < 4 ||
        !hPoly.allFinite() ||
        !std::isfinite(
            duplicateNormalTolerance) ||
        duplicateNormalTolerance < 0.0 ||
        !std::isfinite(
            duplicateOffsetToleranceM) ||
        duplicateOffsetToleranceM < 0.0 ||
        !std::isfinite(
            activityToleranceM) ||
        activityToleranceM < 0.0 ||
        !std::isfinite(
            normalEpsilon) ||
        normalEpsilon <= 0.0)
    {
        return result;
    }


    // --------------------------------------------------------
    // Normalize every H-plane.
    //
    // Positive row scaling then has no effect on either
    // duplicate grouping or LP support distances.
    // --------------------------------------------------------
    Eigen::MatrixX4d normalized(
        rowCount,
        4);


    for (int rowId = 0;
         rowId < rowCount;
         ++rowId)
    {
        const Eigen::Vector3d normal =
            hPoly.row(rowId)
                .head<3>()
                .transpose();

        const double normalNorm =
            normal.norm();


        if (!std::isfinite(normalNorm) ||
            normalNorm <= normalEpsilon)
        {
            return result;
        }


        normalized.row(rowId) =
            hPoly.row(rowId) /
            normalNorm;
    }


    // --------------------------------------------------------
    // Deterministically group duplicate geometric planes.
    //
    // Do NOT use n ~ -n equivalence:
    //
    //     n^T x + d <= 0
    //
    // and
    //
    //    -n^T x - d <= 0
    //
    // are opposite halfspaces.
    // --------------------------------------------------------
    std::vector<
        std::vector<int>>
        groups;


    for (int rowId = 0;
         rowId < rowCount;
         ++rowId)
    {
        bool assigned =
            false;


        for (std::vector<int> &group :
             groups)
        {
            const int representativeId =
                group.front();


            const double normalDelta =
                (
                    normalized
                        .row(rowId)
                        .head<3>() -
                    normalized
                        .row(representativeId)
                        .head<3>())
                    .norm();


            const double offsetDelta =
                std::abs(
                    normalized(
                        rowId,
                        3) -
                    normalized(
                        representativeId,
                        3));


            if (normalDelta <=
                    duplicateNormalTolerance &&
                offsetDelta <=
                    duplicateOffsetToleranceM)
            {
                group.push_back(
                    rowId);

                assigned =
                    true;

                break;
            }
        }


        if (!assigned)
        {
            groups.push_back(
                std::vector<int>{
                    rowId});
        }
    }


    result.unique_plane_groups =
        static_cast<int>(
            groups.size());

    result.duplicate_rows =
        rowCount -
        result.unique_plane_groups;


    if (groups.size() < 4)
    {
        return result;
    }


    double maxRedundantViolationM =
        -std::numeric_limits<double>::
            infinity();

    double minFiniteEffectiveViolationM =
        std::numeric_limits<double>::
            infinity();

    int redundantGroupCount =
        0;

    int effectiveFaceCount =
        0;

    int unboundedCount =
        0;


    // --------------------------------------------------------
    // Test each UNIQUE plane group.
    // --------------------------------------------------------
    for (int groupId = 0;
         groupId <
             static_cast<int>(
                 groups.size());
         ++groupId)
    {
        const std::vector<int> &removedGroup =
            groups[groupId];

        const int representativeId =
            removedGroup.front();


        std::vector<bool>
            removed(
                rowCount,
                false);


        for (const int rowId :
             removedGroup)
        {
            removed[rowId] =
                true;
        }


        const int keptCount =
            rowCount -
            static_cast<int>(
                removedGroup.size());


        Eigen::Matrix<double,
                      Eigen::Dynamic,
                      3>
            A(
                keptCount,
                3);

        Eigen::VectorXd b(
            keptCount);


        int keptRow =
            0;


        for (int rowId = 0;
             rowId < rowCount;
             ++rowId)
        {
            if (removed[rowId])
            {
                continue;
            }


            A.row(keptRow) =
                normalized
                    .row(rowId)
                    .head<3>();

            b(keptRow) =
                -normalized(
                    rowId,
                    3);

            ++keptRow;
        }


        if (keptRow !=
            keptCount)
        {
            return result;
        }


        // maximize:
        //
        //     n^T x + d
        //
        // via:
        //
        //     minimize (-n)^T x.
        const Eigen::Vector3d objective =
            -normalized
                 .row(representativeId)
                 .head<3>()
                 .transpose();


        Eigen::Vector3d optimum =
            Eigen::Vector3d::Zero();


        const double minimum =
            sdlp::linprog<3>(
                objective,
                A,
                b,
                optimum);


        if (std::isnan(minimum))
        {
            return result;
        }


        // ----------------------------------------------------
        // sdlp convention:
        //
        //   -inf : objective unbounded below
        //
        // Here this means support in +n direction is
        // unbounded after removing the plane -> effective.
        //
        //   +inf : infeasible remaining constraints.
        //
        // Removing constraints from an originally feasible
        // corridor should never make it infeasible, so treat
        // this as invalid measurement.
        // ----------------------------------------------------
        if (std::isinf(minimum))
        {
            if (minimum <
                0.0)
            {
                ++effectiveFaceCount;
                ++unboundedCount;

                continue;
            }


            return result;
        }


        const double supportViolationM =
            -minimum +
            normalized(
                representativeId,
                3);


        if (!std::isfinite(
                supportViolationM))
        {
            return result;
        }


        if (supportViolationM >
            activityToleranceM)
        {
            ++effectiveFaceCount;

            minFiniteEffectiveViolationM =
                std::min(
                    minFiniteEffectiveViolationM,
                    supportViolationM);
        }
        else
        {
            ++redundantGroupCount;

            maxRedundantViolationM =
                std::max(
                    maxRedundantViolationM,
                    supportViolationM);
        }
    }


    result.effective_faces =
        effectiveFaceCount;

    result.redundant_plane_groups =
        redundantGroupCount;

    result.unbounded_support_tests =
        unboundedCount;


    if (redundantGroupCount > 0)
    {
        result.max_redundant_violation_m =
            maxRedundantViolationM;
    }


    if (std::isfinite(
            minFiniteEffectiveViolationM))
    {
        result.min_finite_effective_violation_m =
            minFiniteEffectiveViolationM;
    }


    result.valid =
        result.effective_faces +
            result.redundant_plane_groups ==
            result.unique_plane_groups;


    return result;
}

} // namespace gcopter_benchmark