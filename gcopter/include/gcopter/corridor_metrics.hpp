#ifndef GCOPTER_CORRIDOR_METRICS_HPP
#define GCOPTER_CORRIDOR_METRICS_HPP

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>

namespace gcopter_benchmark
{

struct CorridorSeedMetric
{
    bool valid = false;

    double radius_m =
        std::numeric_limits<double>::
            quiet_NaN();
};


struct CorridorOverlapMetric
{
    bool valid = false;

    double radius_m =
        std::numeric_limits<double>::
            quiet_NaN();
};


// ============================================================
// Exact Euclidean tube/capsule radius around segment [a,b]
// contained by one H-polytope.
//
// H-polytope convention:
//
//     n^T x + d <= 0.
// ============================================================
inline CorridorSeedMetric
evaluateSegmentSeedRadius(
    const Eigen::MatrixX4d &hPoly,
    const Eigen::Vector3d &a,
    const Eigen::Vector3d &b,
    const double normalEpsilon =
        1.0e-12)
{
    CorridorSeedMetric result;

    if (hPoly.rows() <= 0 ||
        !hPoly.allFinite() ||
        !a.allFinite() ||
        !b.allFinite())
    {
        return result;
    }

    double minRadius =
        std::numeric_limits<double>::
            infinity();

    for (int faceId = 0;
         faceId < hPoly.rows();
         ++faceId)
    {
        const Eigen::Vector3d normal =
            hPoly.row(faceId)
                .head<3>()
                .transpose();

        const double offset =
            hPoly(faceId, 3);

        const double normalNorm =
            normal.norm();

        if (!std::isfinite(normalNorm) ||
            normalNorm <= normalEpsilon ||
            !std::isfinite(offset))
        {
            return result;
        }

        const double support =
            std::max(
                normal.dot(a) +
                    offset,
                normal.dot(b) +
                    offset);

        const double radius =
            -support /
            normalNorm;

        if (!std::isfinite(radius))
        {
            return result;
        }

        minRadius =
            std::min(
                minRadius,
                radius);
    }

    if (!std::isfinite(minRadius))
    {
        return result;
    }

    result.valid =
        true;

    result.radius_m =
        minRadius;

    return result;
}


// ============================================================
// Guaranteed Euclidean ball radius centered at a FIXED
// junction q and contained in P0 ∩ P1.
//
// For:
//
//     P = {x | n_f^T x + d_f <= 0},
//
// the largest ball B(q, r) contained in P has:
//
//     r = min_f -(n_f^T q + d_f) / ||n_f||.
//
// Therefore for neighboring corridors:
//
//     r_ov(q)
//       = min_{f in P0 union P1}
//           -(n_f^T q + d_f) / ||n_f||.
//
// This is the overlap-radius definition frozen for the paper.
// ============================================================
inline CorridorOverlapMetric
evaluateJunctionOverlapRadius(
    const Eigen::MatrixX4d &hPoly0,
    const Eigen::MatrixX4d &hPoly1,
    const Eigen::Vector3d &junction,
    const double normalEpsilon =
        1.0e-12)
{
    CorridorOverlapMetric result;

    if (hPoly0.rows() <= 0 ||
        hPoly1.rows() <= 0 ||
        !hPoly0.allFinite() ||
        !hPoly1.allFinite() ||
        !junction.allFinite())
    {
        return result;
    }

    double minRadius =
        std::numeric_limits<double>::
            infinity();

    auto accumulatePoly =
        [&](const Eigen::MatrixX4d &hPoly)
        {
            for (int faceId = 0;
                 faceId < hPoly.rows();
                 ++faceId)
            {
                const Eigen::Vector3d normal =
                    hPoly.row(faceId)
                        .head<3>()
                        .transpose();

                const double offset =
                    hPoly(faceId, 3);

                const double normalNorm =
                    normal.norm();

                if (!std::isfinite(
                        normalNorm) ||
                    normalNorm <=
                        normalEpsilon ||
                    !std::isfinite(offset))
                {
                    return false;
                }

                const double radius =
                    -(
                        normal.dot(
                            junction) +
                        offset) /
                    normalNorm;

                if (!std::isfinite(radius))
                {
                    return false;
                }

                minRadius =
                    std::min(
                        minRadius,
                        radius);
            }

            return true;
        };

    if (!accumulatePoly(hPoly0) ||
        !accumulatePoly(hPoly1) ||
        !std::isfinite(minRadius))
    {
        return result;
    }

    result.valid =
        true;

    result.radius_m =
        minRadius;

    return result;
}

} // namespace gcopter_benchmark

#endif