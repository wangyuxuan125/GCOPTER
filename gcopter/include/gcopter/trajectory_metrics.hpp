#ifndef GCOPTER_TRAJECTORY_METRICS_HPP
#define GCOPTER_TRAJECTORY_METRICS_HPP

#include "gcopter/trajectory.hpp"
#include "gcopter/flatness.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>

namespace gcopter_benchmark
{

struct FinalTrajectoryMetrics
{
    bool valid = false;

    int piece_count = 0;

    int flatness_sample_count = 0;

    double max_sample_step_s = 1.0e-3;

    double duration_s =
        std::numeric_limits<double>::
            quiet_NaN();

    double length_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double smoothness_energy =
        std::numeric_limits<double>::
            quiet_NaN();

    double time_weight =
        std::numeric_limits<double>::
            quiet_NaN();

    double time_cost =
        std::numeric_limits<double>::
            quiet_NaN();

    double j_kin =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_velocity_mps =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_acceleration_mps2 =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_body_rate_radps =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_tilt_rad =
        std::numeric_limits<double>::
            quiet_NaN();

    double min_thrust_n =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_thrust_n =
        std::numeric_limits<double>::
            quiet_NaN();
};


inline FinalTrajectoryMetrics
evaluateFinalTrajectoryMetrics(
    const Trajectory<5> &trajectory,
    const double smoothnessEnergy,
    const double timeWeight,
    const double vehicleMass,
    const double gravitationalAcceleration,
    const double horizontalDrag,
    const double verticalDrag,
    const double parasiticDrag,
    const double speedSmoothFactor,
    const double maxSampleStepS =
        1.0e-3)
{
    FinalTrajectoryMetrics result;

    result.piece_count =
        trajectory.getPieceNum();

    result.max_sample_step_s =
        maxSampleStepS;

    result.smoothness_energy =
        smoothnessEnergy;

    result.time_weight =
        timeWeight;

    if (result.piece_count <= 0 ||
        !std::isfinite(
            smoothnessEnergy) ||
        !std::isfinite(
            timeWeight) ||
        !std::isfinite(
            vehicleMass) ||
        vehicleMass <= 0.0 ||
        !std::isfinite(
            gravitationalAcceleration) ||
        !std::isfinite(
            horizontalDrag) ||
        !std::isfinite(
            verticalDrag) ||
        !std::isfinite(
            parasiticDrag) ||
        !std::isfinite(
            speedSmoothFactor) ||
        speedSmoothFactor <= 0.0 ||
        !std::isfinite(
            maxSampleStepS) ||
        maxSampleStepS <= 0.0)
    {
        return result;
    }

    result.duration_s =
        trajectory.getTotalDuration();

    if (!std::isfinite(
            result.duration_s) ||
        result.duration_s <= 0.0)
    {
        return result;
    }

    result.time_cost =
        result.time_weight *
        result.duration_s;

    result.j_kin =
        result.smoothness_energy +
        result.time_cost;

    if (!std::isfinite(
            result.time_cost) ||
        !std::isfinite(
            result.j_kin))
    {
        return result;
    }

    result.length_m =
        0.0;

    result.max_velocity_mps =
        0.0;

    result.max_acceleration_mps2 =
        0.0;

    result.max_body_rate_radps =
        0.0;

    result.max_tilt_rad =
        0.0;

    result.min_thrust_n =
        std::numeric_limits<double>::
            infinity();

    result.max_thrust_n =
        -std::numeric_limits<double>::
            infinity();

    flatness::FlatnessMap flatnessMap;

    flatnessMap.reset(
        vehicleMass,
        gravitationalAcceleration,
        horizontalDrag,
        verticalDrag,
        parasiticDrag,
        speedSmoothFactor);

    for (int pieceId = 0;
         pieceId <
             result.piece_count;
         ++pieceId)
    {
        const auto &piece =
            trajectory[pieceId];

        const double duration =
            piece.getDuration();

        if (!std::isfinite(
                duration) ||
            duration <= 0.0)
        {
            return result;
        }

        // ----------------------------------------------------
        // Exact polynomial extrema for velocity and
        // acceleration.
        // ----------------------------------------------------
        const double pieceMaxVelocity =
            piece.getMaxVelRate();

        const double pieceMaxAcceleration =
            piece.getMaxAccRate();

        if (!std::isfinite(
                pieceMaxVelocity) ||
            !std::isfinite(
                pieceMaxAcceleration))
        {
            return result;
        }

        result.max_velocity_mps =
            std::max(
                result.max_velocity_mps,
                pieceMaxVelocity);

        result.max_acceleration_mps2 =
            std::max(
                result.max_acceleration_mps2,
                pieceMaxAcceleration);

        // ----------------------------------------------------
        // Use an EVEN interval count so the same dense grid
        // supports composite Simpson integration.
        //
        // Actual dt <= maxSampleStepS.
        // ----------------------------------------------------
        int intervalCount =
            std::max(
                2,
                static_cast<int>(
                    std::ceil(
                        duration /
                        maxSampleStepS)));

        if (intervalCount % 2 != 0)
        {
            ++intervalCount;
        }

        const double dt =
            duration /
            static_cast<double>(
                intervalCount);

        double simpsonSpeedSum =
            0.0;

        for (int sampleId = 0;
             sampleId <=
                 intervalCount;
             ++sampleId)
        {
            const double t =
                dt *
                static_cast<double>(
                    sampleId);

            const Eigen::Vector3d vel =
                piece.getVel(t);

            const Eigen::Vector3d acc =
                piece.getAcc(t);

            const Eigen::Vector3d jer =
                piece.getJer(t);

            if (!vel.allFinite() ||
                !acc.allFinite() ||
                !jer.allFinite())
            {
                return result;
            }

            const double speed =
                vel.norm();

            if (!std::isfinite(speed))
            {
                return result;
            }

            const int simpsonWeight =
                (sampleId == 0 ||
                 sampleId ==
                     intervalCount)
                    ? 1
                    : (sampleId % 2 == 0
                           ? 2
                           : 4);

            simpsonSpeedSum +=
                static_cast<double>(
                    simpsonWeight) *
                speed;

            // ------------------------------------------------
            // Flatness-based dynamics.
            //
            // Yaw and yaw rate are fixed to zero because the
            // translational trajectory benchmark does not
            // optimize an independent yaw profile.
            // ------------------------------------------------
            double thrust =
                0.0;

            Eigen::Vector4d quat =
                Eigen::Vector4d::
                    Zero();

            Eigen::Vector3d bodyRate =
                Eigen::Vector3d::
                    Zero();

            flatnessMap.forward(
                vel,
                acc,
                jer,
                0.0,
                0.0,
                thrust,
                quat,
                bodyRate);

            if (!std::isfinite(
                    thrust) ||
                !quat.allFinite() ||
                !bodyRate.allFinite())
            {
                return result;
            }

            const double quatNorm =
                quat.norm();

            if (!std::isfinite(
                    quatNorm) ||
                quatNorm <= 1.0e-12)
            {
                return result;
            }

            const Eigen::Vector4d
                unitQuat =
                    quat /
                    quatNorm;

            const double tiltSinHalf =
                std::min(
                    1.0,
                    std::sqrt(
                        std::max(
                            0.0,
                            unitQuat(1) *
                                    unitQuat(1) +
                                unitQuat(2) *
                                    unitQuat(2))));

            const double tilt =
                2.0 *
                std::asin(
                    tiltSinHalf);

            if (!std::isfinite(tilt))
            {
                return result;
            }

            result.max_body_rate_radps =
                std::max(
                    result.max_body_rate_radps,
                    bodyRate.norm());

            result.max_tilt_rad =
                std::max(
                    result.max_tilt_rad,
                    tilt);

            result.min_thrust_n =
                std::min(
                    result.min_thrust_n,
                    thrust);

            result.max_thrust_n =
                std::max(
                    result.max_thrust_n,
                    thrust);

            ++result
                 .flatness_sample_count;
        }

        result.length_m +=
            dt /
            3.0 *
            simpsonSpeedSum;
    }

    result.valid =
        result.flatness_sample_count > 0 &&
        std::isfinite(
            result.length_m) &&
        result.length_m >= 0.0 &&
        std::isfinite(
            result.max_velocity_mps) &&
        std::isfinite(
            result.max_acceleration_mps2) &&
        std::isfinite(
            result.max_body_rate_radps) &&
        std::isfinite(
            result.max_tilt_rad) &&
        std::isfinite(
            result.min_thrust_n) &&
        std::isfinite(
            result.max_thrust_n);

    return result;
}

} // namespace gcopter_benchmark

#endif