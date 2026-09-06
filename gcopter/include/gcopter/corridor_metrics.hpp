#ifndef GCOPTER_CORRIDOR_METRICS_HPP
#define GCOPTER_CORRIDOR_METRICS_HPP

#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"

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

struct CorridorDirectionalReserveMetric
{
    bool valid = false;

    double positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double symmetric_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double span_m =
        std::numeric_limits<double>::
            quiet_NaN();
};

struct CorridorVolumeMetric
{
    bool valid = false;

    int vertex_count = 0;

    int triangle_count = 0;

    double volume_m3 =
        std::numeric_limits<double>::
            quiet_NaN();

    // Maximum normalized H-plane violation among the
    // enumerated vertices, expressed in meters.
    //
    // <= 0:
    //     strictly inside / on numerical boundary.
    //
    // small positive:
    //     numerical H->V reconstruction error.
    double max_vertex_violation_m =
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

// ============================================================
// Exact directional translation reserve of the protected
// segment capsule:
//
//     K = [a,b] (+) B(protectedRadius)
//
// inside one final H-polytope P.
//
// For a unit direction u:
//
//     positive_m
//         = max t such that K + t u is contained in P
//
//     negative_m
//         = max t such that K - t u is contained in P
//
// symmetric_m = min(positive_m, negative_m)
// span_m      = positive_m + negative_m
//
// H-polytope convention:
//
//     n^T x + d <= 0.
//
// This measurement is invariant to arbitrary face scaling.
// ============================================================
inline CorridorDirectionalReserveMetric
evaluateProtectedSegmentDirectionalReserve(
    const Eigen::MatrixX4d &hPoly,
    const Eigen::Vector3d &a,
    const Eigen::Vector3d &b,
    const double protectedRadiusM,
    const Eigen::Vector3d &direction,
    const double normalEpsilon =
        1.0e-12,
    const double projectionEpsilon =
        1.0e-12)
{
    CorridorDirectionalReserveMetric result;

    if (hPoly.rows() <= 0 ||
        !hPoly.allFinite() ||
        !a.allFinite() ||
        !b.allFinite() ||
        !direction.allFinite() ||
        !std::isfinite(
            protectedRadiusM) ||
        protectedRadiusM < 0.0)
    {
        return result;
    }

    const double directionNorm =
        direction.norm();

    if (!std::isfinite(
            directionNorm) ||
        directionNorm <=
            normalEpsilon)
    {
        return result;
    }

    const Eigen::Vector3d unitDirection =
        direction /
        directionNorm;

    double positiveReserve =
        std::numeric_limits<double>::
            infinity();

    double negativeReserve =
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

        if (!std::isfinite(
                normalNorm) ||
            normalNorm <=
                normalEpsilon ||
            !std::isfinite(offset))
        {
            return result;
        }

        // Normalize first so:
        //
        //   clearance is in meters
        //   projection is dimensionless
        //
        // and arbitrary H-plane scaling cancels.
        const Eigen::Vector3d unitNormal =
            normal /
            normalNorm;

        const double normalizedOffset =
            offset /
            normalNorm;

        // Support of:
        //
        //   [a,b] (+) B(protectedRadiusM)
        //
        // in the current face-normal direction.
        const double capsuleSupport =
            std::max(
                unitNormal.dot(a),
                unitNormal.dot(b)) +
            protectedRadiusM;

        const double clearance =
            -(
                capsuleSupport +
                normalizedOffset);

        if (!std::isfinite(clearance))
        {
            return result;
        }

        const double projection =
            unitNormal.dot(
                unitDirection);

        if (projection >
            projectionEpsilon)
        {
            positiveReserve =
                std::min(
                    positiveReserve,
                    clearance /
                        projection);
        }
        else if (projection <
                 -projectionEpsilon)
        {
            negativeReserve =
                std::min(
                    negativeReserve,
                    clearance /
                        (-projection));
        }
    }

    if (!std::isfinite(
            positiveReserve) ||
        !std::isfinite(
            negativeReserve))
    {
        // A bounded corridor should have a limiting face
        // in both +u and -u directions.
        return result;
    }

    result.valid =
        true;

    result.positive_m =
        positiveReserve;

    result.negative_m =
        negativeReserve;

    result.symmetric_m =
        std::min(
            positiveReserve,
            negativeReserve);

    result.span_m =
        positiveReserve +
        negativeReserve;

    return result;
}

// ============================================================
// Numerical volume of a bounded 3-D convex H-polytope.
//
// Pipeline:
//
//     H representation
//         ->
//     geo_utils::enumerateVs()
//         ->
//     vertex convex hull triangulation
//         ->
//     tetrahedral volume sum.
//
// For a reference point c inside the convex hull:
//
//     V = sum_faces
//           |(a-c) dot ((b-c) x (d-c))| / 6.
//
// The mean of all convex-hull vertices is used as c.
//
// This is a deterministic numerical geometry metric;
// it is NOT labeled as an exact symbolic volume.
// ============================================================
inline CorridorVolumeMetric
evaluateHPolytopeVolume(
    const Eigen::MatrixX4d &hPoly,
    const double vertexEpsilon =
        1.0e-6,
    const double normalEpsilon =
        1.0e-12,
    const double containmentToleranceM =
        1.0e-5,
    const double volumeEpsilonM3 =
        1.0e-12)
{
    CorridorVolumeMetric result;

    if (hPoly.rows() < 4 ||
        !hPoly.allFinite() ||
        !std::isfinite(vertexEpsilon) ||
        vertexEpsilon <= 0.0 ||
        !std::isfinite(normalEpsilon) ||
        normalEpsilon <= 0.0 ||
        !std::isfinite(containmentToleranceM) ||
        containmentToleranceM < 0.0 ||
        !std::isfinite(volumeEpsilonM3) ||
        volumeEpsilonM3 <= 0.0)
    {
        return result;
    }


    // --------------------------------------------------------
    // H -> V.
    // --------------------------------------------------------
    Eigen::Matrix3Xd vertices;

    if (!geo_utils::enumerateVs(
            hPoly,
            vertices,
            vertexEpsilon) ||
        vertices.cols() < 4 ||
        !vertices.allFinite())
    {
        return result;
    }

    result.vertex_count =
        static_cast<int>(
            vertices.cols());


    // --------------------------------------------------------
    // Validate enumerated vertices against the original
    // H-representation with normalized metric distance.
    // --------------------------------------------------------
    double maxVertexViolationM =
        -std::numeric_limits<double>::
            infinity();

    for (int vertexId = 0;
         vertexId < vertices.cols();
         ++vertexId)
    {
        const Eigen::Vector3d vertex =
            vertices.col(
                vertexId);

        for (int faceId = 0;
             faceId < hPoly.rows();
             ++faceId)
        {
            const Eigen::Vector3d normal =
                hPoly.row(faceId)
                    .head<3>()
                    .transpose();

            const double normalNorm =
                normal.norm();

            const double offset =
                hPoly(faceId, 3);

            if (!std::isfinite(normalNorm) ||
                normalNorm <= normalEpsilon ||
                !std::isfinite(offset))
            {
                return result;
            }

            const double violationM =
                (
                    normal.dot(vertex) +
                    offset) /
                normalNorm;

            if (!std::isfinite(
                    violationM))
            {
                return result;
            }

            maxVertexViolationM =
                std::max(
                    maxVertexViolationM,
                    violationM);
        }
    }

    if (!std::isfinite(
            maxVertexViolationM))
    {
        return result;
    }

    result.max_vertex_violation_m =
        maxVertexViolationM;


    // --------------------------------------------------------
    // Triangulate the convex hull of the V-representation.
    //
    // useOriginalIndices=true means the returned index buffer
    // addresses the input vertex matrix directly, matching the
    // existing geo_utils implementation.
    // --------------------------------------------------------
    quickhull::QuickHull<double>
        qh;

    const double qhullEpsilon =
        std::min(
            vertexEpsilon,
            quickhull::
                defaultEps<double>());

    const auto hull =
        qh.getConvexHull(
            vertices.data(),
            vertices.cols(),
            false,
            true,
            qhullEpsilon);

    const auto &indexBuffer =
        hull.getIndexBuffer();

    if (indexBuffer.size() < 12 ||
        indexBuffer.size() % 3 != 0)
    {
        return result;
    }

    result.triangle_count =
        static_cast<int>(
            indexBuffer.size() /
            3);


    // --------------------------------------------------------
    // Convex combination of all vertices. For a full-
    // dimensional bounded convex polytope this is an interior
    // reference point.
    // --------------------------------------------------------
    const Eigen::Vector3d center =
        vertices
            .rowwise()
            .mean();

    if (!center.allFinite())
    {
        return result;
    }


    // --------------------------------------------------------
    // Sum the tetrahedra formed by center and every triangular
    // boundary facet.
    //
    // Absolute value makes the result independent of QuickHull
    // CW / CCW orientation.
    // --------------------------------------------------------
    double volumeM3 =
        0.0;

    for (std::size_t triangleId = 0;
         triangleId <
             indexBuffer.size() /
                 3;
         ++triangleId)
    {
        const std::size_t ia =
            indexBuffer[
                3 * triangleId];

        const std::size_t ib =
            indexBuffer[
                3 * triangleId + 1];

        const std::size_t ic =
            indexBuffer[
                3 * triangleId + 2];

        if (ia >=
                static_cast<std::size_t>(
                    vertices.cols()) ||
            ib >=
                static_cast<std::size_t>(
                    vertices.cols()) ||
            ic >=
                static_cast<std::size_t>(
                    vertices.cols()))
        {
            return result;
        }

        const Eigen::Vector3d a =
            vertices.col(
                static_cast<int>(ia)) -
            center;

        const Eigen::Vector3d b =
            vertices.col(
                static_cast<int>(ib)) -
            center;

        const Eigen::Vector3d c =
            vertices.col(
                static_cast<int>(ic)) -
            center;

        const double tetraVolumeM3 =
            std::abs(
                a.dot(
                    b.cross(c))) /
            6.0;

        if (!std::isfinite(
                tetraVolumeM3))
        {
            return result;
        }

        volumeM3 +=
            tetraVolumeM3;
    }

    if (!std::isfinite(volumeM3) ||
        volumeM3 <= volumeEpsilonM3)
    {
        return result;
    }

    result.volume_m3 =
        volumeM3;

    result.valid =
        maxVertexViolationM <=
            containmentToleranceM;

    return result;
}

} // namespace gcopter_benchmark

#endif