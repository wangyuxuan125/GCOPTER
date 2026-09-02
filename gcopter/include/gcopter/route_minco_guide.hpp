#ifndef ROUTE_MINCO_GUIDE_HPP
#define ROUTE_MINCO_GUIDE_HPP

#include "minco.hpp"
#include "trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_relevant
{

struct RouteMincoGuideOptions
{
    double reference_speed =
        2.0;

    double spatial_sample_step =
        0.0625;

    int min_samples_per_piece =
        8;

    int max_samples_per_piece =
        4096;

    int max_refinement_rounds =
        4;

    double min_piece_time =
        1.0e-3;
};


struct RouteMincoGuideDiagnostics
{
    bool valid =
        false;

    bool collision_free =
        false;

    int initial_waypoint_count =
        0;

    int final_waypoint_count =
        0;

    int initial_piece_count =
        0;

    int final_piece_count =
        0;

    int refinement_rounds =
        0;

    int inserted_waypoint_count =
        0;

    int initial_collision_piece_count =
        0;

    int final_collision_piece_count =
        0;

    int initial_collision_sample_count =
        0;

    int final_collision_sample_count =
        0;

    int final_total_samples =
        0;

    int sample_cap_hits =
        0;

    double build_ms =
        0.0;

    double collision_check_ms =
        0.0;

    double final_max_vel =
        0.0;

    double final_max_acc =
        0.0;
};


// ================================================================
// Build a quintic MINCO trajectory directly through a waypoint
// sequence.  No corridor and no nonlinear optimization are used.
// ================================================================
inline bool buildRouteMincoTrajectory(
    const std::vector<Eigen::Vector3d> &waypoints,
    const RouteMincoGuideOptions &options,
    Trajectory<5> &trajectory,
    double *elapsedMs = nullptr)
{
    trajectory.clear();

    if (waypoints.size() < 2 ||
        options.reference_speed <= 0.0)
    {
        return false;
    }

    const int pieceCount =
        static_cast<int>(
            waypoints.size()) -
        1;

    Eigen::Matrix3Xd innerPoints(
        3,
        std::max(
            pieceCount - 1,
            0));

    for (int i = 1;
         i <
             static_cast<int>(
                 waypoints.size()) -
             1;
         ++i)
    {
        innerPoints.col(i - 1) =
            waypoints[i];
    }

    Eigen::VectorXd times(
        pieceCount);

    for (int i = 0;
         i < pieceCount;
         ++i)
    {
        const double length =
            (waypoints[i + 1] -
             waypoints[i])
                .norm();

        times(i) =
            std::max(
                length /
                    options.reference_speed,
                options.min_piece_time);
    }

    Eigen::Matrix3d headPVA =
        Eigen::Matrix3d::Zero();

    Eigen::Matrix3d tailPVA =
        Eigen::Matrix3d::Zero();

    headPVA.col(0) =
        waypoints.front();

    tailPVA.col(0) =
        waypoints.back();

    const auto started =
        std::chrono::steady_clock::now();

    minco::MINCO_S3NU minco;

    minco.setConditions(
        headPVA,
        tailPVA,
        pieceCount);

    minco.setParameters(
        innerPoints,
        times);

    minco.getTrajectory(
        trajectory);

    if (elapsedMs != nullptr)
    {
        *elapsedMs =
            std::chrono::duration<
                double,
                std::milli>(
                std::chrono::steady_clock::now() -
                started)
                .count();
    }

    if (trajectory.getPieceNum() !=
        pieceCount)
    {
        return false;
    }

    for (int i = 0;
         i < pieceCount;
         ++i)
    {
        if (!trajectory[i]
                 .getCoeffMat()
                 .allFinite() ||
            !std::isfinite(
                trajectory[i]
                    .getDuration()) ||
            trajectory[i]
                    .getDuration() <=
                0.0)
        {
            return false;
        }
    }

    return true;
}


// ================================================================
// Sample collision state of every MINCO piece.
//
// This is a repair trigger, not the final safety certificate.
// Final corridor safety is still checked against the complete map.
// ================================================================
template <typename Map>
inline bool findCollidingRouteMincoPieces(
    const Trajectory<5> &trajectory,
    const Map &map,
    const RouteMincoGuideOptions &options,
    std::vector<int> &collidingPieces,
    int &collisionSamples,
    int &totalSamples,
    int &sampleCapHits,
    double &maxVel,
    double &maxAcc)
{
    collidingPieces.clear();

    collisionSamples =
        0;

    totalSamples =
        0;

    sampleCapHits =
        0;

    maxVel =
        0.0;

    maxAcc =
        0.0;

    for (int pieceId = 0;
         pieceId <
             trajectory.getPieceNum();
         ++pieceId)
    {
        const auto &piece =
            trajectory[pieceId];

        const double pieceMaxVel =
            piece.getMaxVelRate();

        const double pieceMaxAcc =
            piece.getMaxAccRate();

        if (!std::isfinite(pieceMaxVel) ||
            !std::isfinite(pieceMaxAcc))
        {
            return false;
        }

        maxVel =
            std::max(
                maxVel,
                pieceMaxVel);

        maxAcc =
            std::max(
                maxAcc,
                pieceMaxAcc);

        int sampleCount =
            std::max(
                options.min_samples_per_piece,
                static_cast<int>(
                    std::ceil(
                        piece.getDuration() *
                        std::max(
                            pieceMaxVel,
                            1.0e-3) /
                        options.spatial_sample_step)));

        if (sampleCount >
            options.max_samples_per_piece)
        {
            sampleCount =
                options.max_samples_per_piece;

            ++sampleCapHits;
        }

        bool pieceCollision =
            false;

        for (int sampleId = 0;
             sampleId <=
                 sampleCount;
             ++sampleId)
        {
            const double alpha =
                static_cast<double>(
                    sampleId) /
                static_cast<double>(
                    sampleCount);

            const Eigen::Vector3d position =
                piece.getPos(
                    alpha *
                    piece.getDuration());

            ++totalSamples;

            if (map.query(position) !=
                0)
            {
                ++collisionSamples;

                pieceCollision =
                    true;
            }
        }

        if (pieceCollision)
        {
            collidingPieces.push_back(
                pieceId);
        }
    }

    return true;
}


// ================================================================
// Adaptive route-preserving refinement.
//
// Initial:
//     RRT waypoints -> MINCO.
//
// If MINCO piece i collides, insert
//
//     m_i = 0.5 (q_i + q_{i+1})
//
// on that SAME RRT/polyline segment and rebuild.
//
// Thus no additional search is required.
//
// Only colliding pieces are refined.
// ================================================================
template <typename Map>
inline bool buildCollisionAwareRouteMincoGuide(
    const std::vector<Eigen::Vector3d> &route,
    const Map &map,
    const RouteMincoGuideOptions &options,
    Trajectory<5> &trajectory,
    std::vector<Eigen::Vector3d> &refinedWaypoints,
    RouteMincoGuideDiagnostics *diagnostics =
        nullptr)
{
    RouteMincoGuideDiagnostics local;

    local.initial_waypoint_count =
        static_cast<int>(
            route.size());

    local.initial_piece_count =
        std::max(
            static_cast<int>(
                route.size()) -
                1,
            0);

    refinedWaypoints =
        route;

    std::vector<int>
        collidingPieces;

    for (int round = 0;
         round <=
             options.max_refinement_rounds;
         ++round)
    {
        double buildMs =
            0.0;

        if (!buildRouteMincoTrajectory(
                refinedWaypoints,
                options,
                trajectory,
                &buildMs))
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    local;
            }

            return false;
        }

        local.build_ms +=
            buildMs;

        int collisionSamples =
            0;

        int totalSamples =
            0;

        int sampleCapHits =
            0;

        double maxVel =
            0.0;

        double maxAcc =
            0.0;

        const auto checkStarted =
            std::chrono::steady_clock::now();

        if (!findCollidingRouteMincoPieces(
                trajectory,
                map,
                options,
                collidingPieces,
                collisionSamples,
                totalSamples,
                sampleCapHits,
                maxVel,
                maxAcc))
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    local;
            }

            return false;
        }

        local.collision_check_ms +=
            std::chrono::duration<
                double,
                std::milli>(
                std::chrono::steady_clock::now() -
                checkStarted)
                .count();

        if (round == 0)
        {
            local.initial_collision_piece_count =
                static_cast<int>(
                    collidingPieces.size());

            local.initial_collision_sample_count =
                collisionSamples;
        }

        local.final_collision_piece_count =
            static_cast<int>(
                collidingPieces.size());

        local.final_collision_sample_count =
            collisionSamples;

        local.final_total_samples =
            totalSamples;

        local.sample_cap_hits +=
            sampleCapHits;

        local.final_max_vel =
            maxVel;

        local.final_max_acc =
            maxAcc;

        if (collidingPieces.empty())
        {
            local.valid =
                true;

            local.collision_free =
                true;

            local.final_waypoint_count =
                static_cast<int>(
                    refinedWaypoints.size());

            local.final_piece_count =
                trajectory.getPieceNum();

            if (diagnostics != nullptr)
            {
                *diagnostics =
                    local;
            }

            return true;
        }

        if (round ==
            options.max_refinement_rounds)
        {
            break;
        }

        std::vector<bool> splitPiece(
            trajectory.getPieceNum(),
            false);

        for (const int pieceId :
             collidingPieces)
        {
            if (pieceId >= 0 &&
                pieceId <
                    static_cast<int>(
                        splitPiece.size()))
            {
                splitPiece[
                    pieceId] =
                    true;
            }
        }

        std::vector<Eigen::Vector3d>
            nextWaypoints;

        nextWaypoints.reserve(
            refinedWaypoints.size() +
            collidingPieces.size());

        for (int pieceId = 0;
             pieceId <
                 static_cast<int>(
                     refinedWaypoints.size()) -
                     1;
             ++pieceId)
        {
            nextWaypoints.push_back(
                refinedWaypoints[
                    pieceId]);

            if (splitPiece[
                    pieceId])
            {
                const Eigen::Vector3d midpoint =
                    0.5 *
                    (refinedWaypoints[
                         pieceId] +
                     refinedWaypoints[
                         pieceId + 1]);

                nextWaypoints.push_back(
                    midpoint);

                ++local
                      .inserted_waypoint_count;
            }
        }

        nextWaypoints.push_back(
            refinedWaypoints.back());

        refinedWaypoints.swap(
            nextWaypoints);

        ++local.refinement_rounds;
    }

    local.valid =
        trajectory.getPieceNum() > 0;

    local.collision_free =
        false;

    local.final_waypoint_count =
        static_cast<int>(
            refinedWaypoints.size());

    local.final_piece_count =
        trajectory.getPieceNum();

    if (diagnostics != nullptr)
    {
        *diagnostics =
            local;
    }

    return false;
}

} // namespace traj_relevant

#endif