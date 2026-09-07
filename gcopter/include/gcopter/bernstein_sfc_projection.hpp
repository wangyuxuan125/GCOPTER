#ifndef BERNSTEIN_SFC_PROJECTION_HPP
#define BERNSTEIN_SFC_PROJECTION_HPP

#include "gcopter/minco_affine_map.hpp"

#include <Eigen/Eigen>

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

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

} // namespace traj_relevant

#endif