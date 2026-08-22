#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/experiment_logger.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/traj_favorable_sfc.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    bool experimentLogEnabled;
    std::string experimentLogDirectory;
    std::string experimentTag;
    std::string corridorMethod;
    bool allowCorridorFallback;
    int tfSfcDirectionMode;
    int tfSfcSamplesPerSegment;
    double tfSfcSafetyMargin;
    double tfSfcMaxInflationDistance;
    double tfSfcInflationStep;
    double tfSfcMinOverlapRadius;
    double tfSfcMaxSegmentLength;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.param("Experiment/LogEnabled", experimentLogEnabled, true);
        nh_priv.param<std::string>("Experiment/LogDirectory", experimentLogDirectory,
                                   "/tmp/tf_sfc_results/gcopter");
        nh_priv.param<std::string>("Experiment/Tag", experimentTag, "default");
        nh_priv.param<std::string>("Corridor/Method", corridorMethod, "firi");
        nh_priv.param("Corridor/AllowFallback", allowCorridorFallback, false);
        nh_priv.param("TfSfc/DirectionMode", tfSfcDirectionMode, 1);
        nh_priv.param("TfSfc/SamplesPerSegment", tfSfcSamplesPerSegment, 5);
        nh_priv.param("TfSfc/SafetyMargin", tfSfcSafetyMargin, 0.05);
        nh_priv.param("TfSfc/MaxInflationDistance", tfSfcMaxInflationDistance, 1.0);
        nh_priv.param("TfSfc/InflationStep", tfSfcInflationStep, 0.10);
        nh_priv.param("TfSfc/MinOverlapRadius", tfSfcMinOverlapRadius, 0.04);
        nh_priv.param("TfSfc/MaxSegmentLength", tfSfcMaxSegmentLength, 1.0);
    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;
    gcopter_experiment::CsvLogger experimentLogger;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh),
          experimentLogger(config.experimentLogEnabled,
                           config.experimentLogDirectory,
                           config.experimentTag)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            const auto totalStarted = std::chrono::steady_clock::now();
            gcopter_experiment::RunRecord record;
            std::vector<gcopter_experiment::CorridorRecord> corridorRecords;
            record.run_id = experimentLogger.makeRunId();
            record.experiment_tag = experimentLogger.experimentTag();
            record.requested_method = config.corridorMethod;
            record.method = config.corridorMethod;
            record.timestamp_s = ros::Time::now().toSec();
            auto finishRecord = [&](const std::string &status, const bool success)
            {
                record.status = status;
                record.success = success;
                record.total_planning_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - totalStarted)
                        .count();
                if (!experimentLogger.log(record, corridorRecords))
                {
                    ROS_ERROR_THROTTLE(1.0, "Failed to append GCOPTER experiment CSV in %s",
                                       config.experimentLogDirectory.c_str());
                }
            };

            std::vector<Eigen::Vector3d> route;
            const auto pathStarted = std::chrono::steady_clock::now();
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            record.path_search_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - pathStarted)
                    .count();
            record.route_point_count = static_cast<int>(route.size());
            if (route.size() <= 1)
            {
                finishRecord("path_search_failure", false);
                return;
            }

            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);
            record.map_point_count = static_cast<int>(pc.size());

            const auto corridorStarted = std::chrono::steady_clock::now();
            auto buildFiriCorridors = [&]()
            {
                sfc_gen::convexCover(route,
                                     pc,
                                     voxelMap.getOrigin(),
                                     voxelMap.getCorner(),
                                     7.0,
                                     3.0,
                                     hPolys);
                sfc_gen::shortCut(hPolys);
                corridorRecords.clear();
                for (int i = 0; i < static_cast<int>(hPolys.size()); ++i)
                {
                    gcopter_experiment::CorridorRecord corridorRecord;
                    corridorRecord.piece_id = i;
                    corridorRecord.face_count = hPolys[i].rows();
                    corridorRecord.valid = true;
                    corridorRecords.push_back(corridorRecord);
                }
            };

            bool corridorOk = true;
            if (config.corridorMethod == "firi")
            {
                record.method = "firi";
                buildFiriCorridors();
            }
            else if (config.corridorMethod == "tf_sfc")
            {
                record.method = "tf_sfc";
                Eigen::Matrix<double, 6, 4> boundary =
                    Eigen::Matrix<double, 6, 4>::Zero();
                const Eigen::Vector3d low = voxelMap.getOrigin();
                const Eigen::Vector3d high = voxelMap.getCorner();
                boundary(0, 0) = 1.0;  boundary(0, 3) = -high.x();
                boundary(1, 0) = -1.0; boundary(1, 3) = low.x();
                boundary(2, 1) = 1.0;  boundary(2, 3) = -high.y();
                boundary(3, 1) = -1.0; boundary(3, 3) = low.y();
                boundary(4, 2) = 1.0;  boundary(4, 3) = -high.z();
                boundary(5, 2) = -1.0; boundary(5, 3) = low.z();

                tf_sfc::Parameters parameters;
                parameters.direction_mode = static_cast<tf_sfc::DirectionMode>(
                    std::max(0, std::min(config.tfSfcDirectionMode, 2)));
                parameters.safety_margin = std::max(config.tfSfcSafetyMargin, 0.0);
                parameters.max_inflation_distance =
                    std::max(config.tfSfcMaxInflationDistance, 0.0);
                parameters.inflation_step = std::max(config.tfSfcInflationStep, 1.0e-3);

                const int sampleCount = std::max(config.tfSfcSamplesPerSegment, 2);
                const double obstacleRange = parameters.safety_margin +
                                             parameters.max_inflation_distance +
                                             config.voxelWidth;
                std::vector<Eigen::Vector3d> refinedRoute;
                refinedRoute.push_back(route.front());
                const double maxSegmentLength = std::max(config.tfSfcMaxSegmentLength,
                                                         config.voxelWidth);
                for (int routeId = 0; routeId + 1 < static_cast<int>(route.size()); ++routeId)
                {
                    const Eigen::Vector3d delta = route[routeId + 1] - route[routeId];
                    const int divisions = std::max(
                        1, static_cast<int>(std::ceil(delta.norm() / maxSegmentLength)));
                    for (int division = 1; division <= divisions; ++division)
                    {
                        refinedRoute.push_back(
                            route[routeId] + delta * static_cast<double>(division) /
                                                 static_cast<double>(divisions));
                    }
                }
                std::vector<tf_sfc::Corridor,
                            Eigen::aligned_allocator<tf_sfc::Corridor>> generatedCorridors;
                for (int pieceId = 0;
                     pieceId + 1 < static_cast<int>(refinedRoute.size()); ++pieceId)
                {
                    const Eigen::Vector3d a = refinedRoute[pieceId];
                    const Eigen::Vector3d b = refinedRoute[pieceId + 1];
                    Eigen::Matrix3Xd samples(3, sampleCount);
                    for (int sampleId = 0; sampleId < sampleCount; ++sampleId)
                    {
                        const double alpha = static_cast<double>(sampleId) /
                                             static_cast<double>(sampleCount - 1);
                        samples.col(sampleId) = (1.0 - alpha) * a + alpha * b;
                    }

                    const Eigen::Vector3d localMin = a.cwiseMin(b).array() - obstacleRange;
                    const Eigen::Vector3d localMax = a.cwiseMax(b).array() + obstacleRange;
                    std::vector<Eigen::Vector3d> localPoints;
                    for (const Eigen::Vector3d &point : pc)
                    {
                        if ((point.array() >= localMin.array()).all() &&
                            (point.array() <= localMax.array()).all())
                        {
                            localPoints.push_back(point);
                        }
                    }
                    Eigen::Matrix3Xd obstaclePoints(3, localPoints.size());
                    for (int pointId = 0; pointId < static_cast<int>(localPoints.size()); ++pointId)
                    {
                        obstaclePoints.col(pointId) = localPoints[pointId];
                    }

                    const Eigen::Vector3d tangent = b - a;
                    const double verticalAlignment = tangent.norm() > 1.0e-9
                                                         ? std::abs(tangent.normalized().dot(
                                                               Eigen::Vector3d::UnitZ()))
                                                         : 0.0;
                    const Eigen::Vector3d lateral =
                        verticalAlignment < 0.9
                            ? Eigen::Vector3d::UnitZ()
                            : Eigen::Vector3d::UnitY();
                    tf_sfc::Corridor corridor;
                    corridor.piece_id = pieceId;
                    const auto pieceStarted = std::chrono::steady_clock::now();
                    const bool generated = tf_sfc::generateCorridor(
                        boundary, obstaclePoints, samples, tangent, lateral,
                        corridor, parameters);
                    corridor.piece_id = pieceId;

                    gcopter_experiment::CorridorRecord corridorRecord;
                    corridorRecord.piece_id = pieceId;
                    corridorRecord.face_count = corridor.face_num;
                    corridorRecord.generation_time_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - pieceStarted)
                            .count();
                    corridorRecord.weighted_width = corridor.weighted_width;
                    corridorRecord.min_sample_slack = corridor.min_sample_slack;
                    corridorRecord.valid = generated;
                    corridorRecord.direction_fallback = corridor.direction_fallback;
                    corridorRecord.failure_reason =
                        tf_sfc::failureReasonName(corridor.failure_reason);
                    corridorRecords.push_back(corridorRecord);
                    if (!generated)
                    {
                        corridorOk = false;
                        break;
                    }

                    if (!generatedCorridors.empty())
                    {
                        const double overlap = tf_sfc::overlapRadiusAtPoint(
                            generatedCorridors.back(), corridor, a);
                        corridorRecords[corridorRecords.size() - 2].overlap_radius_to_next = overlap;
                        if (overlap + 1.0e-9 < config.tfSfcMinOverlapRadius)
                        {
                            corridorRecords.back().valid = false;
                            corridorRecords.back().failure_reason = "overlap_too_small";
                            corridorOk = false;
                            break;
                        }
                    }
                    hPolys.push_back(corridor.hpoly);
                    generatedCorridors.push_back(corridor);
                }
                if (corridorOk && hPolys.size() == 1)
                {
                    hPolys.push_back(hPolys.front());
                    corridorRecords.push_back(corridorRecords.front());
                    corridorRecords.back().piece_id = 1;
                }

                if (!corridorOk && config.allowCorridorFallback)
                {
                    record.fallback_used = true;
                    record.method = "firi";
                    hPolys.clear();
                    buildFiriCorridors();
                    corridorOk = true;
                }
            }
            else
            {
                corridorOk = false;
            }
            record.corridor_generation_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - corridorStarted)
                    .count();
            record.corridor_count = static_cast<int>(hPolys.size());
            for (const Eigen::MatrixX4d &hPoly : hPolys)
            {
                record.total_faces += static_cast<int>(hPoly.rows());
            }
            record.mean_faces = hPolys.empty()
                                    ? 0.0
                                    : static_cast<double>(record.total_faces) /
                                          static_cast<double>(hPolys.size());
            if (!corridorOk)
            {
                finishRecord(config.corridorMethod == "tf_sfc"
                                 ? "tf_sfc_generation_failure"
                                 : "invalid_corridor_method",
                             false);
                return;
            }

            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                const auto setupStarted = std::chrono::steady_clock::now();
                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    record.optimizer_setup_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - setupStarted)
                            .count();
                    finishRecord("setup_failure", false);
                    return;
                }
                record.optimizer_setup_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - setupStarted)
                        .count();

                const auto optimizerStarted = std::chrono::steady_clock::now();
                record.final_cost = gcopter.optimize(traj, config.relCostTol);
                record.optimizer_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - optimizerStarted)
                        .count();
                if (!std::isfinite(record.final_cost))
                {
                    finishRecord("optimizer_failure", false);
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    record.trajectory_piece_count = traj.getPieceNum();
                    record.trajectory_duration_s = traj.getTotalDuration();
                    const int lengthSamples = std::max(20, 20 * traj.getPieceNum());
                    Eigen::Vector3d previous = traj.getPos(0.0);
                    for (int i = 1; i <= lengthSamples; ++i)
                    {
                        const Eigen::Vector3d current = traj.getPos(
                            record.trajectory_duration_s * static_cast<double>(i) /
                            static_cast<double>(lengthSamples));
                        record.trajectory_length_m += (current - previous).norm();
                        previous = current;
                    }
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route);
                    finishRecord("success", true);
                    return;
                }
                finishRecord("empty_trajectory", false);
                return;
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
