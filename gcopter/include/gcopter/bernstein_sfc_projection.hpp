#ifndef BERNSTEIN_SFC_PROJECTION_HPP
#define BERNSTEIN_SFC_PROJECTION_HPP

#include "gcopter/minco_affine_map.hpp"

#include <Eigen/Eigen>
#include <algorithm>
#include <array>

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

struct BernsteinAffineControlPoint
{
    Eigen::Vector3d offset =
        Eigen::Vector3d::Zero();

    Eigen::VectorXd beta;
};


using BernsteinAffineControlPolygon =
    std::array<
        BernsteinAffineControlPoint,
        6>;


struct LocalBernsteinCut
{
    bool valid = false;

    bool fixed_infeasible = false;

    int piece = -1;
    int face = -1;
    int depth = 0;
    int leaf = -1;

    double interval_begin = 0.0;
    double interval_end = 1.0;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};


inline bool
buildBernsteinAffineControlPolygon(
    const MincoWaypointAffineMap &affineMap,
    const int pieceId,
    BernsteinAffineControlPolygon &controls)
{
    if (!affineMap.valid() ||
        pieceId < 0 ||
        pieceId >= affineMap.pieceCount())
    {
        return false;
    }

    for (int controlId = 0;
         controlId < 6;
         ++controlId)
    {
        if (!affineMap
                 .bernsteinControlAffineCoefficients(
                     pieceId,
                     controlId,
                     controls[controlId].offset,
                     controls[controlId].beta))
        {
            return false;
        }

        if (!controls[controlId]
                 .offset.allFinite() ||
            !controls[controlId]
                 .beta.allFinite())
        {
            return false;
        }
    }

    return true;
}

inline bool
splitBernsteinAffineControlPolygonHalf(
    const BernsteinAffineControlPolygon &parent,
    BernsteinAffineControlPolygon &left,
    BernsteinAffineControlPolygon &right)
{
    const int betaSize =
        parent[0].beta.size();

    for (int controlId = 0;
         controlId < 6;
         ++controlId)
    {
        if (parent[controlId].beta.size() !=
                betaSize ||
            !parent[controlId]
                 .offset.allFinite() ||
            !parent[controlId]
                 .beta.allFinite())
        {
            return false;
        }
    }

    BernsteinAffineControlPoint
        work[6][6];

    for (int i = 0;
         i < 6;
         ++i)
    {
        work[0][i] =
            parent[i];
    }

    left[0] =
        parent[0];

    right[5] =
        parent[5];

    for (int level = 1;
         level < 6;
         ++level)
    {
        for (int i = 0;
             i < 6 - level;
             ++i)
        {
            work[level][i].offset =
                0.5 *
                (
                    work[level - 1][i]
                        .offset +
                    work[level - 1][i + 1]
                        .offset
                );

            work[level][i].beta =
                0.5 *
                (
                    work[level - 1][i]
                        .beta +
                    work[level - 1][i + 1]
                        .beta
                );
        }

        left[level] =
            work[level][0];

        right[5 - level] =
            work[level][5 - level];
    }

    for (int i = 0;
         i < 6;
         ++i)
    {
        if (!left[i].offset.allFinite() ||
            !left[i].beta.allFinite() ||
            !right[i].offset.allFinite() ||
            !right[i].beta.allFinite())
        {
            return false;
        }
    }

    return true;
}

inline int
bernsteinLeafForNormalizedTime(
    const double normalizedTime,
    const int depth)
{
    if (!std::isfinite(normalizedTime) ||
        depth < 0 ||
        depth > 20)
    {
        return -1;
    }

    const double tau =
        std::max(
            0.0,
            std::min(
                1.0,
                normalizedTime));

    const int leafCount =
        1 << depth;

    if (tau >= 1.0)
    {
        return leafCount - 1;
    }

    return std::min(
        static_cast<int>(
            std::floor(
                tau *
                static_cast<double>(
                    leafCount))),
        leafCount - 1);
}


inline bool
extractBernsteinAffineLeaf(
    const BernsteinAffineControlPolygon &whole,
    const int depth,
    const int leafId,
    BernsteinAffineControlPolygon &leaf)
{
    if (depth < 0 ||
        depth > 20)
    {
        return false;
    }

    const int leafCount =
        1 << depth;

    if (leafId < 0 ||
        leafId >= leafCount)
    {
        return false;
    }

    BernsteinAffineControlPolygon current =
        whole;

    for (int level = 0;
         level < depth;
         ++level)
    {
        BernsteinAffineControlPolygon left;
        BernsteinAffineControlPolygon right;

        if (!splitBernsteinAffineControlPolygonHalf(
                current,
                left,
                right))
        {
            return false;
        }

        // Binary representation of leafId selects
        // left/right from the most significant level
        // to the least significant level.
        const int bit =
            (leafId >>
             (depth - 1 - level)) &
            1;

        current =
            bit == 0
                ? left
                : right;
    }

    leaf =
        current;

    return true;
}


inline LocalBernsteinCut
buildLocalBernsteinCut(
    const MincoWaypointAffineMap &affineMap,
    const std::vector<Eigen::MatrixX4d> &corridors,
    const int pieceId,
    const int faceId,
    const double normalizedTime,
    const int subdivisionDepth)
{
    LocalBernsteinCut result;

    if (!affineMap.valid() ||
        pieceId < 0 ||
        pieceId >= affineMap.pieceCount() ||
        pieceId >=
            static_cast<int>(corridors.size()) ||
        subdivisionDepth < 0)
    {
        return result;
    }

    const auto &poly =
        corridors[pieceId];

    if (poly.cols() != 4 ||
        faceId < 0 ||
        faceId >= poly.rows() ||
        !poly.allFinite())
    {
        return result;
    }

    const int leafId =
        bernsteinLeafForNormalizedTime(
            normalizedTime,
            subdivisionDepth);

    if (leafId < 0)
    {
        return result;
    }

    BernsteinAffineControlPolygon whole;

    if (!buildBernsteinAffineControlPolygon(
            affineMap,
            pieceId,
            whole))
    {
        return result;
    }

    BernsteinAffineControlPolygon leaf;

    if (!extractBernsteinAffineLeaf(
            whole,
            subdivisionDepth,
            leafId,
            leaf))
    {
        return result;
    }

    const Eigen::Vector3d normal =
        poly.block<1, 3>(
                faceId,
                0)
            .transpose();

    const double planeOffset =
        poly(faceId, 3);

    if (!normal.allFinite() ||
        !std::isfinite(planeOffset) ||
        normal.norm() <= 1.0e-12)
    {
        return result;
    }

    const int variableDimension =
        affineMap.variableDimension();

    std::vector<Eigen::VectorXd>
        rows;

    std::vector<double>
        rhsValues;

    rows.reserve(6);
    rhsValues.reserve(6);

    for (int controlId = 0;
         controlId < 6;
         ++controlId)
    {
        Eigen::VectorXd row =
            Eigen::VectorXd::Zero(
                variableDimension);

        for (int waypointId = 0;
             waypointId <
                 leaf[controlId]
                     .beta.size();
             ++waypointId)
        {
            row.segment<3>(
                3 * waypointId) =
                leaf[controlId]
                    .beta(waypointId) *
                normal;
        }

        double rhs =
            -planeOffset -
            normal.dot(
                leaf[controlId]
                    .offset);

        const double rowNorm =
            row.norm();

        if (!std::isfinite(rowNorm) ||
            !std::isfinite(rhs))
        {
            return result;
        }

        if (rowNorm <= 1.0e-12)
        {
            if (rhs < -1.0e-10)
            {
                result.fixed_infeasible =
                    true;

                return result;
            }

            continue;
        }

        row /= rowNorm;
        rhs /= rowNorm;

        rows.push_back(row);
        rhsValues.push_back(rhs);
    }

    result.A.resize(
        static_cast<int>(rows.size()),
        variableDimension);

    result.b.resize(
        static_cast<int>(rows.size()));

    for (int i = 0;
         i < static_cast<int>(
             rows.size());
         ++i)
    {
        result.A.row(i) =
            rows[i].transpose();

        result.b(i) =
            rhsValues[i];
    }

    const int leafCount =
        1 << subdivisionDepth;

    result.piece =
        pieceId;

    result.face =
        faceId;

    result.depth =
        subdivisionDepth;

    result.leaf =
        leafId;

    result.interval_begin =
        static_cast<double>(
            leafId) /
        static_cast<double>(
            leafCount);

    result.interval_end =
        static_cast<double>(
            leafId + 1) /
        static_cast<double>(
            leafCount);

    result.valid =
        result.A.allFinite() &&
        result.b.allFinite();

    return result;
}

inline bool
evaluateLocalBernsteinFaceBoundM(
    const MincoWaypointAffineMap &affineMap,
    const std::vector<Eigen::MatrixX4d> &corridors,
    const Eigen::Matrix3Xd &innerPoints,
    const int pieceId,
    const int faceId,
    const double normalizedTime,
    const int subdivisionDepth,
    double &maxViolationM,
    int &leafId,
    double &intervalBegin,
    double &intervalEnd)
{
    maxViolationM =
        -std::numeric_limits<double>::
            infinity();

    leafId = -1;

    intervalBegin = 0.0;
    intervalEnd = 1.0;

    if (!affineMap.valid() ||
        innerPoints.rows() != 3 ||
        innerPoints.cols() !=
            affineMap.waypointCount() ||
        !innerPoints.allFinite() ||
        pieceId < 0 ||
        pieceId >= affineMap.pieceCount() ||
        pieceId >=
            static_cast<int>(
                corridors.size()) ||
        subdivisionDepth < 0 ||
        subdivisionDepth > 20)
    {
        return false;
    }

    const auto &poly =
        corridors[pieceId];

    if (poly.cols() != 4 ||
        faceId < 0 ||
        faceId >= poly.rows() ||
        !poly.allFinite())
    {
        return false;
    }

    leafId =
        bernsteinLeafForNormalizedTime(
            normalizedTime,
            subdivisionDepth);

    if (leafId < 0)
    {
        return false;
    }

    BernsteinAffineControlPolygon whole;

    if (!buildBernsteinAffineControlPolygon(
            affineMap,
            pieceId,
            whole))
    {
        return false;
    }

    BernsteinAffineControlPolygon leaf;

    if (!extractBernsteinAffineLeaf(
            whole,
            subdivisionDepth,
            leafId,
            leaf))
    {
        return false;
    }

    const Eigen::Vector3d normal =
        poly.block<1, 3>(
                faceId,
                0)
            .transpose();

    const double planeOffset =
        poly(faceId, 3);

    const double normalNorm =
        normal.norm();

    if (!normal.allFinite() ||
        !std::isfinite(planeOffset) ||
        !std::isfinite(normalNorm) ||
        normalNorm <= 1.0e-12)
    {
        return false;
    }

    for (int controlId = 0;
         controlId < 6;
         ++controlId)
    {
        if (leaf[controlId].beta.size() !=
                affineMap.waypointCount() ||
            !leaf[controlId]
                 .offset.allFinite() ||
            !leaf[controlId]
                 .beta.allFinite())
        {
            return false;
        }

        Eigen::Vector3d controlPoint =
            leaf[controlId].offset;

        for (int waypointId = 0;
             waypointId <
                 affineMap.waypointCount();
             ++waypointId)
        {
            controlPoint +=
                leaf[controlId]
                    .beta(waypointId) *
                innerPoints.col(
                    waypointId);
        }

        const double violationM =
            (normal.dot(controlPoint) +
             planeOffset) /
            normalNorm;

        if (!std::isfinite(
                violationM))
        {
            return false;
        }

        maxViolationM =
            std::max(
                maxViolationM,
                violationM);
    }

    const int leafCount =
        1 << subdivisionDepth;

    intervalBegin =
        static_cast<double>(
            leafId) /
        static_cast<double>(
            leafCount);

    intervalEnd =
        static_cast<double>(
            leafId + 1) /
        static_cast<double>(
            leafCount);

    return std::isfinite(
        maxViolationM);
}

inline bool
subdivideBernsteinAffineControlPolygon(
    const BernsteinAffineControlPolygon &parent,
    const int depth,
    std::vector<
        BernsteinAffineControlPolygon> &leaves)
{
    if (depth < 0)
    {
        return false;
    }

    if (depth == 0)
    {
        leaves.push_back(
            parent);

        return true;
    }

    BernsteinAffineControlPolygon
        left;

    BernsteinAffineControlPolygon
        right;

    if (!splitBernsteinAffineControlPolygonHalf(
            parent,
            left,
            right))
    {
        return false;
    }

    return
        subdivideBernsteinAffineControlPolygon(
            left,
            depth - 1,
            leaves) &&
        subdivideBernsteinAffineControlPolygon(
            right,
            depth - 1,
            leaves);
}


struct BernsteinSfcConstraintSet
{
    bool valid = false;

    bool fixed_infeasible = false;

    int variable_dimension = 0;

    int constraint_count = 0;

    int skipped_fixed_constraints = 0;

    Eigen::MatrixXd A;

    Eigen::VectorXd b;

    double assembly_ms = 0.0;
};


inline BernsteinSfcConstraintSet
buildBernsteinSfcConstraintSet(
    const MincoWaypointAffineMap &affineMap,
    const std::vector<Eigen::MatrixX4d> &corridors)
{
    BernsteinSfcConstraintSet result;

    const auto started =
        std::chrono::steady_clock::now();

    if (!affineMap.valid() ||
        affineMap.pieceCount() !=
            static_cast<int>(corridors.size()))
    {
        return result;
    }

    result.variable_dimension =
        affineMap.variableDimension();

    std::vector<Eigen::VectorXd> rows;
    std::vector<double> rhsValues;

    std::size_t estimatedRows = 0;

    for (const auto &poly : corridors)
    {
        estimatedRows +=
            static_cast<std::size_t>(
                6 * poly.rows());
    }

    rows.reserve(estimatedRows);
    rhsValues.reserve(estimatedRows);

    for (int pieceId = 0;
         pieceId < affineMap.pieceCount();
         ++pieceId)
    {
        const auto &poly =
            corridors[pieceId];

        if (poly.cols() != 4 ||
            !poly.allFinite())
        {
            return result;
        }

        for (int faceId = 0;
             faceId < poly.rows();
             ++faceId)
        {
            const Eigen::Vector3d normal =
                poly.block<1, 3>(
                        faceId,
                        0)
                    .transpose();

            const double planeOffset =
                poly(faceId, 3);

            if (!normal.allFinite() ||
                !std::isfinite(planeOffset) ||
                normal.norm() <= 1.0e-12)
            {
                return result;
            }

            for (int controlId = 0;
                 controlId < 6;
                 ++controlId)
            {
                Eigen::VectorXd row;
                double rhs = 0.0;

                if (!affineMap
                         .buildBernsteinHalfspaceConstraintRow(
                             pieceId,
                             controlId,
                             normal,
                             planeOffset,
                             row,
                             rhs))
                {
                    return result;
                }

                const double rowNorm =
                    row.norm();

                if (!std::isfinite(rowNorm))
                {
                    return result;
                }

                // Constraint independent of internal waypoints.
                //
                //     0 <= rhs
                //
                // must already hold because only the fixed boundary
                // states can affect this Bernstein control point.
                if (rowNorm <= 1.0e-12)
                {
                    if (rhs < -1.0e-10)
                    {
                        result.fixed_infeasible = true;
                        return result;
                    }

                    ++result.skipped_fixed_constraints;
                    continue;
                }

                // Unit-row normalization improves QP conditioning.
                row /= rowNorm;
                rhs /= rowNorm;

                rows.push_back(row);
                rhsValues.push_back(rhs);
            }
        }
    }

    result.A.resize(
        static_cast<int>(rows.size()),
        result.variable_dimension);

    result.b.resize(
        static_cast<int>(rows.size()));

    for (int rowId = 0;
         rowId < static_cast<int>(rows.size());
         ++rowId)
    {
        result.A.row(rowId) =
            rows[rowId].transpose();

        result.b(rowId) =
            rhsValues[rowId];
    }

    result.constraint_count =
        static_cast<int>(rows.size());

    result.valid =
        result.A.allFinite() &&
        result.b.allFinite();

    result.assembly_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() -
            started)
            .count();

    return result;
}

inline BernsteinSfcConstraintSet
buildSubdividedBernsteinSfcConstraintSet(
    const MincoWaypointAffineMap &affineMap,
    const std::vector<Eigen::MatrixX4d> &corridors,
    const int subdivisionDepth)
{
    BernsteinSfcConstraintSet result;

    const auto started =
        std::chrono::steady_clock::now();

    if (!affineMap.valid() ||
        subdivisionDepth < 0 ||
        affineMap.pieceCount() !=
            static_cast<int>(
                corridors.size()))
    {
        return result;
    }

    result.variable_dimension =
        affineMap.variableDimension();

    std::vector<Eigen::VectorXd>
        rows;

    std::vector<double>
        rhsValues;

    const std::size_t leafCount =
        static_cast<std::size_t>(
            1ULL <<
            subdivisionDepth);

    std::size_t estimatedRows =
        0;

    for (const auto &poly :
         corridors)
    {
        estimatedRows +=
            leafCount *
            6ULL *
            static_cast<std::size_t>(
                poly.rows());
    }

    rows.reserve(
        estimatedRows);

    rhsValues.reserve(
        estimatedRows);

    for (int pieceId = 0;
         pieceId <
             affineMap.pieceCount();
         ++pieceId)
    {
        const auto &poly =
            corridors[pieceId];

        if (poly.cols() != 4 ||
            !poly.allFinite())
        {
            return result;
        }

        BernsteinAffineControlPolygon
            whole;

        if (!buildBernsteinAffineControlPolygon(
                affineMap,
                pieceId,
                whole))
        {
            return result;
        }

        std::vector<
            BernsteinAffineControlPolygon>
                leaves;

        leaves.reserve(
            leafCount);

        if (!subdivideBernsteinAffineControlPolygon(
                whole,
                subdivisionDepth,
                leaves))
        {
            return result;
        }

        if (leaves.size() !=
            leafCount)
        {
            return result;
        }

        for (const auto &leaf :
             leaves)
        {
            for (int faceId = 0;
                 faceId <
                     poly.rows();
                 ++faceId)
            {
                const Eigen::Vector3d
                    normal =
                        poly.block<1, 3>(
                                faceId,
                                0)
                            .transpose();

                const double
                    planeOffset =
                        poly(
                            faceId,
                            3);

                const double
                    normalNorm =
                        normal.norm();

                if (!normal.allFinite() ||
                    !std::isfinite(
                        planeOffset) ||
                    !std::isfinite(
                        normalNorm) ||
                    normalNorm <=
                        1.0e-12)
                {
                    return result;
                }

                for (int controlId = 0;
                     controlId < 6;
                     ++controlId)
                {
                    const auto &control =
                        leaf[
                            controlId];

                    Eigen::VectorXd row =
                        Eigen::VectorXd::Zero(
                            result
                                .variable_dimension);

                    for (int waypointId = 0;
                         waypointId <
                             control.beta.size();
                         ++waypointId)
                    {
                        row.segment<3>(
                            3 *
                            waypointId) =
                            control.beta(
                                waypointId) *
                            normal;
                    }

                    double rhs =
                        -planeOffset -
                        normal.dot(
                            control.offset);

                    const double
                        rowNorm =
                            row.norm();

                    if (!std::isfinite(
                            rowNorm) ||
                        !std::isfinite(
                            rhs))
                    {
                        return result;
                    }

                    if (rowNorm <=
                        1.0e-12)
                    {
                        if (rhs <
                            -1.0e-10)
                        {
                            result
                                .fixed_infeasible =
                                    true;

                            return result;
                        }

                        ++result
                              .skipped_fixed_constraints;

                        continue;
                    }

                    row /=
                        rowNorm;

                    rhs /=
                        rowNorm;

                    rows.push_back(
                        row);

                    rhsValues.push_back(
                        rhs);
                }
            }
        }
    }

    result.A.resize(
        static_cast<int>(
            rows.size()),
        result.variable_dimension);

    result.b.resize(
        static_cast<int>(
            rows.size()));

    for (int rowId = 0;
         rowId <
             static_cast<int>(
                 rows.size());
         ++rowId)
    {
        result.A.row(
            rowId) =
                rows[rowId]
                    .transpose();

        result.b(
            rowId) =
                rhsValues[
                    rowId];
    }

    result.constraint_count =
        static_cast<int>(
            rows.size());

    result.valid =
        result.A.allFinite() &&
        result.b.allFinite();

    result.assembly_ms =
        std::chrono::duration<
            double,
            std::milli>(
                std::chrono::
                    steady_clock::now() -
                started)
            .count();

    return result;
}

} // namespace traj_relevant

#endif