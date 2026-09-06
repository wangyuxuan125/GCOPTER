#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#ifdef GCOPTER_WITH_DECOMP_UTIL
#include <decomp_util/line_segment.h>
#endif


namespace gcopter_benchmark
{

struct ControlledRilsCorridorInfo
{
    bool valid = false;

    int corridor_id = -1;

    int local_obstacle_count = 0;

    // Faces returned by DecompUtil:
    //
    //   obstacle tangent planes
    //   +
    //   six segment-frame local bbox planes.
    int generator_faces = 0;

    int obstacle_faces = 0;

    int local_domain_faces = 0;

    // We append the six finite GCOPTER map-domain planes
    // after the DecompUtil output.
    int map_domain_faces = 0;

    int final_rows = 0;

    double seed_max_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double dilate_ms =
        std::numeric_limits<double>::
            quiet_NaN();
};


struct ControlledRilsBuildResult
{
    bool available = false;

    bool success = false;

    bool mapping_valid = false;

    double range_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double total_ms =
        std::numeric_limits<double>::
            quiet_NaN();

    std::vector<Eigen::MatrixX4d>
        hpolys;

    std::vector<ControlledRilsCorridorInfo>
        infos;
};


inline ControlledRilsBuildResult
buildControlledRilsCorridors(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<Eigen::Vector3d> &obstacles,
    const Eigen::Vector3d &mapLow,
    const Eigen::Vector3d &mapHigh,
    const double rangeM)
{
    ControlledRilsBuildResult result;

#ifdef GCOPTER_WITH_DECOMP_UTIL

    result.available = true;
    result.range_m = rangeM;


    if (route.size() < 2 ||
        !std::isfinite(rangeM) ||
        rangeM <= 0.0 ||
        !mapLow.allFinite() ||
        !mapHigh.allFinite() ||
        (mapHigh.array() <=
         mapLow.array()).any())
    {
        return result;
    }


    // --------------------------------------------------------
    // Convert the common GCOPTER obstacle cloud once.
    // --------------------------------------------------------
    vec_Vec3f decompObstacles;

    decompObstacles.reserve(
        obstacles.size());


    for (const Eigen::Vector3d &point :
         obstacles)
    {
        if (!point.allFinite())
        {
            continue;
        }

        Vec3f p;

        p <<
            point.x(),
            point.y(),
            point.z();

        decompObstacles.push_back(p);
    }


    const int segmentCount =
        static_cast<int>(
            route.size()) - 1;


    result.hpolys.reserve(
        segmentCount);

    result.infos.reserve(
        segmentCount);


    const auto totalStarted =
        std::chrono::
            steady_clock::now();


    for (int segmentId = 0;
         segmentId < segmentCount;
         ++segmentId)
    {
        ControlledRilsCorridorInfo info;

        info.corridor_id =
            segmentId;


        const Eigen::Vector3d &seedA =
            route[segmentId];

        const Eigen::Vector3d &seedB =
            route[segmentId + 1];


        if (!seedA.allFinite() ||
            !seedB.allFinite() ||
            (seedB - seedA).norm() <=
                1.0e-12)
        {
            result.infos.push_back(info);
            return result;
        }


        Vec3f a;
        Vec3f b;

        a <<
            seedA.x(),
            seedA.y(),
            seedA.z();

        b <<
            seedB.x(),
            seedB.y(),
            seedB.z();


        // ====================================================
        // Direct use of the Liu/DecompUtil reference primitive.
        //
        // No route subdivision:
        //
        //   one ORIGINAL route segment
        //       ->
        //   one LineSegment3D
        //       ->
        //   one RILS polyhedron.
        // ====================================================
        LineSegment3D line(
            a,
            b);


        // Controlled Geometry:
        //
        // same 3 m local budget as Controlled FIRI,
        // expressed in RILS' own segment frame.
        Vec3f localBBox;

        localBBox <<
            rangeM,
            rangeM,
            rangeM;


        line.set_local_bbox(
            localBBox);


        // DecompBase::set_obs() performs the reference
        // local-bbox obstacle filtering.
        line.set_obs(
            decompObstacles);


        info.local_obstacle_count =
            static_cast<int>(
                line.get_obs().size());


        const auto dilateStarted =
            std::chrono::
                steady_clock::now();


        // offset_x = 0:
        //
        // keep the reference line-segment ellipsoid;
        // do NOT enlarge the longitudinal semi-axis.
        line.dilate(
            0.0);


        info.dilate_ms =
            std::chrono::duration<
                double,
                std::milli>(
                    std::chrono::
                        steady_clock::now() -
                    dilateStarted)
                .count();


        const Polyhedron3D polyhedron =
            line.get_polyhedron();


        const vec_E<Hyperplane3D> planes =
            polyhedron.hyperplanes();


        // LineSegment3D::dilate() always appends six
        // local-bbox planes when localBBox != 0.
        if (planes.size() < 6)
        {
            result.infos.push_back(info);
            return result;
        }


        info.generator_faces =
            static_cast<int>(
                planes.size());

        info.local_domain_faces =
            6;

        info.obstacle_faces =
            std::max(
                0,
                info.generator_faces -
                    info.local_domain_faces);


        // ====================================================
        // Convert:
        //
        //     Hyperplane(p,n)
        //
        // into GCOPTER convention
        //
        //     n^T x + d <= 0
        //
        // with normalized n.
        //
        // Add six finite-map domain rows afterward.
        // ====================================================
        Eigen::MatrixX4d hpoly(
            info.generator_faces + 6,
            4);


        int row = 0;


        for (const Hyperplane3D &plane :
             planes)
        {
            Eigen::Vector3d normal(
                plane.n_.x(),
                plane.n_.y(),
                plane.n_.z());


            const double normalNorm =
                normal.norm();


            if (!normal.allFinite() ||
                !std::isfinite(normalNorm) ||
                normalNorm <= 1.0e-12)
            {
                result.infos.push_back(info);
                return result;
            }


            normal /=
                normalNorm;


            const Eigen::Vector3d point(
                plane.p_.x(),
                plane.p_.y(),
                plane.p_.z());


            if (!point.allFinite())
            {
                result.infos.push_back(info);
                return result;
            }


            hpoly.row(row).head<3>() =
                normal.transpose();

            hpoly(row, 3) =
                -normal.dot(point);

            ++row;
        }


        // Finite GCOPTER map domain.
        hpoly.row(row++) <<
            1.0, 0.0, 0.0,
            -mapHigh.x();

        hpoly.row(row++) <<
            -1.0, 0.0, 0.0,
            mapLow.x();

        hpoly.row(row++) <<
            0.0, 1.0, 0.0,
            -mapHigh.y();

        hpoly.row(row++) <<
            0.0, -1.0, 0.0,
            mapLow.y();

        hpoly.row(row++) <<
            0.0, 0.0, 1.0,
            -mapHigh.z();

        hpoly.row(row++) <<
            0.0, 0.0, -1.0,
            mapLow.z();


        info.map_domain_faces =
            6;

        info.final_rows =
            static_cast<int>(
                hpoly.rows());


        if (row != hpoly.rows() ||
            !hpoly.allFinite())
        {
            result.infos.push_back(info);
            return result;
        }


        // ====================================================
        // Deterministic seed containment sanity check.
        //
        // All rows have unit normals, so a positive
        // violation is directly measured in metres.
        // ====================================================
        const Eigen::Vector4d seedAH(
            seedA.x(),
            seedA.y(),
            seedA.z(),
            1.0);

        const Eigen::Vector4d seedBH(
            seedB.x(),
            seedB.y(),
            seedB.z(),
            1.0);


        const double seedViolationA =
            (hpoly * seedAH)
                .maxCoeff();

        const double seedViolationB =
            (hpoly * seedBH)
                .maxCoeff();


        info.seed_max_violation_m =
            std::max(
                0.0,
                std::max(
                    seedViolationA,
                    seedViolationB));


        info.valid =
            std::isfinite(
                info.seed_max_violation_m) &&
            info.seed_max_violation_m <=
                1.0e-8;


        result.hpolys.push_back(
            hpoly);

        result.infos.push_back(
            info);


        if (!info.valid)
        {
            return result;
        }
    }


    result.total_ms =
        std::chrono::duration<
            double,
            std::milli>(
                std::chrono::
                    steady_clock::now() -
                totalStarted)
            .count();


    result.mapping_valid =
        static_cast<int>(
            result.hpolys.size()) ==
            segmentCount &&
        static_cast<int>(
            result.infos.size()) ==
            segmentCount;


    result.success =
        result.mapping_valid;


    if (result.success)
    {
        for (const ControlledRilsCorridorInfo &info :
             result.infos)
        {
            if (!info.valid)
            {
                result.success =
                    false;

                break;
            }
        }
    }

#else

    (void)route;
    (void)obstacles;
    (void)mapLow;
    (void)mapHigh;
    (void)rangeM;

#endif

    return result;
}

} // namespace gcopter_benchmark