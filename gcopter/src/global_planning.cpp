#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/experiment_logger.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/traj_favorable_sfc.hpp"

#ifdef GCOPTER_WITH_DECOMP_UTIL
#include <decomp_util/ellipsoid_decomp.h>
#endif

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <cstdint>
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
    int mapSeed;
    int routeSeed;
    bool fixedStartGoalEnabled;
    double fixedStartX;
    double fixedStartY;
    double fixedStartZ;
    double fixedGoalX;
    double fixedGoalY;
    double fixedGoalZ;
    std::string corridorMethod;
    bool allowCorridorFallback;
    int tfSfcDirectionMode;
    int tfSfcSamplesPerSegment;
    int tfSfcMaxFaces;
    int tfSfcMaxObsFaces;
    double tfSfcSafetyMargin;
    double tfSfcMaxInflationDistance;
    double tfSfcInflationStep;
    double tfSfcMinOverlapRadius;
    double tfSfcMaxSegmentLength;
    int tfFiriMaxFaces;
    double tfFiriDirectionalWidthWeight;
    double tfFiriFaceCountWeight;
    int tfFiriCandidatePoolSize;
    double tfFiriProgress;
    double tfFiriRange;
    double decompLocalBBoxForward;
    double decompLocalBBoxLateral;
    double decompLocalBBoxVertical;
    double decompMaxSegmentLength;
    double decompMinOverlapRadius;

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
        nh_priv.param("Experiment/MapSeed", mapSeed, 1024);
        nh_priv.param("Experiment/RouteSeed", routeSeed, 0);
        nh_priv.param("Experiment/FixedStartGoalEnabled", fixedStartGoalEnabled, false);
        nh_priv.param("Experiment/FixedStartX", fixedStartX, 0.0);
        nh_priv.param("Experiment/FixedStartY", fixedStartY, 0.0);
        nh_priv.param("Experiment/FixedStartZ", fixedStartZ, 1.0);
        nh_priv.param("Experiment/FixedGoalX", fixedGoalX, 0.0);
        nh_priv.param("Experiment/FixedGoalY", fixedGoalY, 0.0);
        nh_priv.param("Experiment/FixedGoalZ", fixedGoalZ, 1.0);
        nh_priv.param<std::string>("Corridor/Method", corridorMethod, "firi");
        nh_priv.param("Corridor/AllowFallback", allowCorridorFallback, false);
        nh_priv.param("TfSfc/DirectionMode", tfSfcDirectionMode, 1);
        nh_priv.param("TfSfc/SamplesPerSegment", tfSfcSamplesPerSegment, 5);
        nh_priv.param("TfSfc/MaxFaces", tfSfcMaxFaces, 12);
        nh_priv.param("TfSfc/MaxObsFaces", tfSfcMaxObsFaces, 6);
        nh_priv.param("TfSfc/SafetyMargin", tfSfcSafetyMargin, 0.05);
        nh_priv.param("TfSfc/MaxInflationDistance", tfSfcMaxInflationDistance, 1.0);
        nh_priv.param("TfSfc/InflationStep", tfSfcInflationStep, 0.10);
        nh_priv.param("TfSfc/MinOverlapRadius", tfSfcMinOverlapRadius, 0.04);
        nh_priv.param("TfSfc/MaxSegmentLength", tfSfcMaxSegmentLength, 1.0);
        nh_priv.param("TfFiri/MaxFaces", tfFiriMaxFaces, 24);
        nh_priv.param("TfFiri/DirectionalWidthWeight", tfFiriDirectionalWidthWeight, 1.0);
        nh_priv.param("TfFiri/FaceCountWeight", tfFiriFaceCountWeight, 0.25);
        nh_priv.param("TfFiri/CandidatePoolSize", tfFiriCandidatePoolSize, 8);
        nh_priv.param("TfFiri/Progress", tfFiriProgress, 7.0);
        nh_priv.param("TfFiri/Range", tfFiriRange, 3.0);
        nh_priv.param("Decomp/LocalBBoxForward", decompLocalBBoxForward, 0.5);
        nh_priv.param("Decomp/LocalBBoxLateral", decompLocalBBoxLateral, 3.0);
        nh_priv.param("Decomp/LocalBBoxVertical", decompLocalBBoxVertical, 3.0);
        nh_priv.param("Decomp/MaxSegmentLength", decompMaxSegmentLength, 3.0);
        nh_priv.param("Decomp/MinOverlapRadius", decompMinOverlapRadius, 0.01);
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
    bool fixedPlanTriggered;
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
          fixedPlanTriggered(false),
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
            if (config.fixedStartGoalEnabled && !fixedPlanTriggered)
            {
                fixedPlanTriggered = true;
                const Eigen::Vector3d start(config.fixedStartX,
                                            config.fixedStartY,
                                            config.fixedStartZ);
                const Eigen::Vector3d goal(config.fixedGoalX,
                                           config.fixedGoalY,
                                           config.fixedGoalZ);
                if (voxelMap.query(start) != 0 || voxelMap.query(goal) != 0)
                {
                    ROS_ERROR("Fixed experiment start or goal is occupied after dilation.");
                    return;
                }
                if ((goal - start).norm() <= config.voxelWidth)
                {
                    ROS_ERROR("Fixed experiment start and goal must be distinct.");
                    return;
                }
                startGoal.clear();
                startGoal.push_back(start);
                startGoal.push_back(goal);
                visualizer.visualizeStartGoal(start, 0.5, 0);
                visualizer.visualizeStartGoal(goal, 0.5, 1);
                plan();
            }
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
            record.map_seed = config.mapSeed;
            record.route_seed = config.routeSeed;
            record.fixed_start_goal = config.fixedStartGoalEnabled;
            record.start_x = startGoal[0].x();
            record.start_y = startGoal[0].y();
            record.start_z = startGoal[0].z();
            record.goal_x = startGoal[1].x();
            record.goal_y = startGoal[1].y();
            record.goal_z = startGoal[1].z();
            record.voxel_width_m = config.voxelWidth;
            record.dilate_radius_m = config.dilateRadius;
            record.route_timeout_s = config.timeoutRRT;
            record.max_velocity_mps = config.maxVelMag;
            record.max_body_rate_radps = config.maxBdrMag;
            record.max_tilt_rad = config.maxTiltAngle;
            record.min_thrust = config.minThrust;
            record.max_thrust = config.maxThrust;
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
                                                   &voxelMap, config.timeoutRRT,
                                                   route,
                                                   static_cast<std::uint_fast32_t>(
                                                       std::max(config.routeSeed, 0)));
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

            auto buildTfFiriCorridors = [&]() -> bool
            {
                firi::TrajectoryFavorableOptions options;
                options.enabled = true;
                options.directional_width_weight =
                    std::max(0.0, config.tfFiriDirectionalWidthWeight);
                options.face_count_weight =
                    std::max(0.0, config.tfFiriFaceCountWeight);
                options.candidate_pool_size =
                    std::max(1, config.tfFiriCandidatePoolSize);
                options.max_faces = std::max(6, config.tfFiriMaxFaces);
                std::vector<sfc_gen::TrajectoryFavorableFiriInfo> infos;
                const auto tfFiriStarted = std::chrono::steady_clock::now();
                const bool generated = sfc_gen::trajectoryFavorableConvexCover(
                    route, pc, voxelMap.getOrigin(), voxelMap.getCorner(),
                    std::max(config.tfFiriProgress, config.voxelWidth),
                    std::max(config.tfFiriRange, config.voxelWidth),
                    options, hPolys, infos);
                const double generationMs =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - tfFiriStarted)
                        .count();
                corridorRecords.clear();
                if (!generated)
                {
                    const double perPieceGenerationMs =
                        generationMs /
                        static_cast<double>(std::max<std::size_t>(infos.size(), 1));
                    const std::size_t completedCount =
                        std::min(hPolys.size(), infos.size());
                    for (std::size_t i = 0; i < completedCount; ++i)
                    {
                        const sfc_gen::TrajectoryFavorableFiriInfo &info = infos[i];
                        gcopter_experiment::CorridorRecord completedRecord;
                        completedRecord.piece_id = static_cast<int>(i);
                        completedRecord.face_count = hPolys[i].rows();
                        completedRecord.face_budget_saturated =
                            info.face_budget_saturated;
                        completedRecord.unresolved_constraint_count =
                            info.unresolved_constraint_count;
                        completedRecord.unresolved_boundary_count =
                            info.unresolved_boundary_count;
                        completedRecord.unresolved_obstacle_count =
                            info.unresolved_obstacle_count;
                        completedRecord.budget_exchange_attempted =
                            info.budget_exchange_attempted;
                        completedRecord.budget_exchange_accepted =
                            info.budget_exchange_accepted;
                        completedRecord.generation_time_ms = perPieceGenerationMs;
                        completedRecord.weighted_width =
                            2.0 * info.directional_radius;
                        completedRecord.directional_radius_m =
                            info.directional_radius;
                        completedRecord.directional_width_weight =
                            options.directional_width_weight;
                        completedRecord.face_count_weight =
                            options.face_count_weight;
                        completedRecord.valid = hPolys[i].allFinite() &&
                                                hPolys[i].rows() <= options.max_faces;
                        completedRecord.failure_reason =
                            completedRecord.valid ? "none" : "face_budget_or_numeric_failure";
                        if (i > 0)
                        {
                            corridorRecords.back().overlap_radius_to_next =
                                geo_utils::overlap(hPolys[i - 1], hPolys[i], 0.01)
                                    ? 0.01
                                    : 0.0;
                        }
                        corridorRecords.push_back(completedRecord);
                    }
                    if (!infos.empty())
                    {
                        const sfc_gen::TrajectoryFavorableFiriInfo &failure =
                            infos.back();
                        gcopter_experiment::CorridorRecord corridorRecord;
                        corridorRecord.piece_id = static_cast<int>(hPolys.size());
                        corridorRecord.face_count = failure.face_count;
                        corridorRecord.face_budget_saturated =
                            failure.face_budget_saturated;
                        corridorRecord.unresolved_constraint_count =
                            failure.unresolved_constraint_count;
                        corridorRecord.unresolved_boundary_count =
                            failure.unresolved_boundary_count;
                        corridorRecord.unresolved_obstacle_count =
                            failure.unresolved_obstacle_count;
                        corridorRecord.budget_exchange_attempted =
                            failure.budget_exchange_attempted;
                        corridorRecord.budget_exchange_accepted =
                            failure.budget_exchange_accepted;
                        corridorRecord.generation_time_ms = perPieceGenerationMs;
                        corridorRecord.directional_radius_m =
                            failure.directional_radius;
                        corridorRecord.directional_width_weight =
                            options.directional_width_weight;
                        corridorRecord.face_count_weight =
                            options.face_count_weight;
                        corridorRecord.valid = false;
                        corridorRecord.failure_reason =
                            failure.face_budget_saturated
                                ? "face_budget_exhausted"
                                : "tf_firi_generation_failure";
                        corridorRecords.push_back(corridorRecord);
                    }
                    return false;
                }
                if (hPolys.size() != infos.size())
                {
                    return false;
                }
                for (int i = 0; i < static_cast<int>(hPolys.size()); ++i)
                {
                    gcopter_experiment::CorridorRecord corridorRecord;
                    corridorRecord.piece_id = i;
                    corridorRecord.face_count = hPolys[i].rows();
                    corridorRecord.face_budget_saturated =
                        infos[i].face_budget_saturated;
                    corridorRecord.unresolved_constraint_count =
                        infos[i].unresolved_constraint_count;
                    corridorRecord.unresolved_boundary_count =
                        infos[i].unresolved_boundary_count;
                    corridorRecord.unresolved_obstacle_count =
                        infos[i].unresolved_obstacle_count;
                    corridorRecord.budget_exchange_attempted =
                        infos[i].budget_exchange_attempted;
                    corridorRecord.budget_exchange_accepted =
                        infos[i].budget_exchange_accepted;
                    corridorRecord.generation_time_ms =
                        generationMs / static_cast<double>(hPolys.size());
                    corridorRecord.weighted_width =
                        2.0 * infos[i].directional_radius;
                    corridorRecord.directional_radius_m =
                        infos[i].directional_radius;
                    corridorRecord.directional_width_weight =
                        options.directional_width_weight;
                    corridorRecord.face_count_weight =
                        options.face_count_weight;
                    corridorRecord.valid = hPolys[i].allFinite() &&
                                           hPolys[i].rows() <= options.max_faces;
                    corridorRecord.failure_reason =
                        corridorRecord.valid ? "none" : "face_budget_or_numeric_failure";
                    if (!corridorRecord.valid)
                    {
                        corridorRecords.push_back(corridorRecord);
                        return false;
                    }
                    if (i > 0)
                    {
                        const bool overlap = geo_utils::overlap(
                            hPolys[i - 1], hPolys[i], 0.01);
                        corridorRecords.back().overlap_radius_to_next =
                            overlap ? 0.01 : 0.0;
                        if (!overlap)
                        {
                            corridorRecord.valid = false;
                            corridorRecord.failure_reason = "overlap_failure";
                            corridorRecords.push_back(corridorRecord);
                            return false;
                        }
                    }
                    corridorRecords.push_back(corridorRecord);
                }
                return !hPolys.empty();
            };

            auto buildEllipsoidDecompCorridors = [&]() -> bool
            {
#ifdef GCOPTER_WITH_DECOMP_UTIL
                vec_Vec3f decompPath;
                if (route.empty())
                {
                    return false;
                }
                decompPath.push_back(route.front());
                const double maxSegmentLength = std::max(
                    config.decompMaxSegmentLength, config.voxelWidth);
                for (int routeId = 0; routeId + 1 < static_cast<int>(route.size()); ++routeId)
                {
                    const Eigen::Vector3d delta = route[routeId + 1] - route[routeId];
                    const int divisions = std::max(
                        1, static_cast<int>(std::ceil(delta.norm() / maxSegmentLength)));
                    for (int division = 1; division <= divisions; ++division)
                    {
                        const Eigen::Vector3d point =
                            route[routeId] + delta * static_cast<double>(division) /
                                                 static_cast<double>(divisions);
                        if ((point - decompPath.back()).norm() > 1.0e-6)
                        {
                            decompPath.push_back(point);
                        }
                    }
                }
                if (decompPath.size() < 2)
                {
                    return false;
                }

                vec_Vec3f obstacles;
                obstacles.reserve(pc.size());
                for (const Eigen::Vector3d &point : pc)
                {
                    obstacles.push_back(point);
                }

                EllipsoidDecomp3D decomp;
                decomp.set_obs(obstacles);
                Vec3f localBBox;
                localBBox << std::max(config.decompLocalBBoxForward, config.voxelWidth),
                             std::max(config.decompLocalBBoxLateral, config.voxelWidth),
                             std::max(config.decompLocalBBoxVertical, config.voxelWidth);
                decomp.set_local_bbox(localBBox);
                const auto decompStarted = std::chrono::steady_clock::now();
                decomp.dilate(decompPath);
                const double decompMs = std::chrono::duration<double, std::milli>(
                                            std::chrono::steady_clock::now() - decompStarted)
                                            .count();
                const vec_E<Polyhedron3D> polyhedrons = decomp.get_polyhedrons();
                if (polyhedrons.size() + 1 != decompPath.size())
                {
                    return false;
                }

                hPolys.clear();
                corridorRecords.clear();
                const Eigen::Vector3d low = voxelMap.getOrigin();
                const Eigen::Vector3d high = voxelMap.getCorner();
                for (int pieceId = 0; pieceId < static_cast<int>(polyhedrons.size()); ++pieceId)
                {
                    const vec_E<Hyperplane3D> planes = polyhedrons[pieceId].hyperplanes();
                    Eigen::MatrixX4d hpoly(planes.size() + 6, 4);
                    int row = 0;
                    for (const Hyperplane3D &plane : planes)
                    {
                        Eigen::Vector3d normal = plane.n_;
                        const double norm = normal.norm();
                        if (!normal.allFinite() || norm <= 1.0e-9)
                        {
                            return false;
                        }
                        normal /= norm;
                        hpoly.row(row).head<3>() = normal.transpose();
                        hpoly(row++, 3) = -normal.dot(plane.p_);
                    }
                    hpoly.row(row++) << 1.0, 0.0, 0.0, -high.x();
                    hpoly.row(row++) << -1.0, 0.0, 0.0, low.x();
                    hpoly.row(row++) << 0.0, 1.0, 0.0, -high.y();
                    hpoly.row(row++) << 0.0, -1.0, 0.0, low.y();
                    hpoly.row(row++) << 0.0, 0.0, 1.0, -high.z();
                    hpoly.row(row++) << 0.0, 0.0, -1.0, low.z();

                    const Eigen::Vector4d ah(decompPath[pieceId].x(),
                                             decompPath[pieceId].y(),
                                             decompPath[pieceId].z(), 1.0);
                    const Eigen::Vector4d bh(decompPath[pieceId + 1].x(),
                                             decompPath[pieceId + 1].y(),
                                             decompPath[pieceId + 1].z(), 1.0);
                    const bool valid = hpoly.allFinite() &&
                                       (hpoly * ah).maxCoeff() <= 1.0e-6 &&
                                       (hpoly * bh).maxCoeff() <= 1.0e-6;
                    gcopter_experiment::CorridorRecord corridorRecord;
                    corridorRecord.piece_id = pieceId;
                    corridorRecord.face_count = hpoly.rows();
                    corridorRecord.generation_time_ms =
                        decompMs / static_cast<double>(polyhedrons.size());
                    auto pointSlack = [](const Eigen::MatrixX4d &poly,
                                         const Eigen::Vector3d &point)
                    {
                        double slack = std::numeric_limits<double>::infinity();
                        for (int faceId = 0; faceId < poly.rows(); ++faceId)
                        {
                            const double norm = poly.row(faceId).head<3>().norm();
                            slack = std::min(slack,
                                             -(poly.row(faceId).head<3>().dot(point) +
                                               poly(faceId, 3)) /
                                                 norm);
                        }
                        return slack;
                    };
                    const Eigen::Vector3d midpoint =
                        0.5 * (decompPath[pieceId] + decompPath[pieceId + 1]);
                    corridorRecord.min_sample_slack = std::min(
                        pointSlack(hpoly, decompPath[pieceId]),
                        std::min(pointSlack(hpoly, midpoint),
                                 pointSlack(hpoly, decompPath[pieceId + 1])));
                    corridorRecord.weighted_width =
                        2.0 * corridorRecord.min_sample_slack;
                    corridorRecord.valid = valid;
                    corridorRecord.failure_reason = valid ? "none" : "seed_outside_corridor";
                    corridorRecords.push_back(corridorRecord);
                    if (!valid)
                    {
                        return false;
                    }
                    if (!hPolys.empty())
                    {
                        const double overlap = std::min(
                            pointSlack(hPolys.back(), decompPath[pieceId]),
                            pointSlack(hpoly, decompPath[pieceId]));
                        corridorRecords[corridorRecords.size() - 2]
                            .overlap_radius_to_next = overlap;
                        if (overlap + 1.0e-9 < config.decompMinOverlapRadius)
                        {
                            corridorRecords.back().valid = false;
                            corridorRecords.back().failure_reason = "overlap_too_small";
                            return false;
                        }
                    }
                    hPolys.push_back(hpoly);
                }
                if (hPolys.size() == 1)
                {
                    hPolys.push_back(hPolys.front());
                    corridorRecords.push_back(corridorRecords.front());
                    corridorRecords.back().piece_id = 1;
                }
                return !hPolys.empty();
#else
                ROS_ERROR_THROTTLE(
                    1.0,
                    "Corridor/Method=ellipsoid_decomp requested, but DecompUtil was not found at build time.");
                return false;
#endif
            };

            bool corridorOk = true;
            if (config.corridorMethod == "firi")
            {
                record.method = "firi";
                buildFiriCorridors();
            }
            else if (config.corridorMethod == "tf_firi")
            {
                record.method = "tf_firi";
                corridorOk = buildTfFiriCorridors();
                if (!corridorOk && config.allowCorridorFallback)
                {
                    record.fallback_used = true;
                    record.method = "firi";
                    hPolys.clear();
                    buildFiriCorridors();
                    corridorOk = true;
                }
            }
            else if (config.corridorMethod == "ellipsoid_decomp")
            {
                record.method = "ellipsoid_decomp";
                corridorOk = buildEllipsoidDecompCorridors();
                if (!corridorOk && config.allowCorridorFallback)
                {
                    record.fallback_used = true;
                    record.method = "firi";
                    hPolys.clear();
                    buildFiriCorridors();
                    corridorOk = true;
                }
            }
            else if (config.corridorMethod == "tf_sfc" ||
                     config.corridorMethod == "obb")
            {
                record.method = config.corridorMethod;
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
                parameters.max_faces = std::max(config.tfSfcMaxFaces, 6);
                parameters.max_obs_faces =
                    std::max(0, std::min(config.tfSfcMaxObsFaces,
                                         parameters.max_faces - 6));
                parameters.enable_obstacle_planes =
                    config.corridorMethod == "tf_sfc";
                parameters.safety_margin = std::max(config.tfSfcSafetyMargin, 0.0);
                parameters.max_inflation_distance =
                    std::max(config.tfSfcMaxInflationDistance, 0.0);
                parameters.inflation_step = std::max(config.tfSfcInflationStep, 1.0e-3);
                parameters.min_overlap_radius =
                    std::max(config.tfSfcMinOverlapRadius, 0.0);

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
                    corridorRecord.obstacle_face_count =
                        corridor.obstacle_face_num;
                    corridorRecord.obstacle_point_count =
                        corridor.obstacle_point_num;
                    corridorRecord.face_budget_saturated =
                        corridor.face_budget_saturated;
                    corridorRecord.generation_time_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - pieceStarted)
                            .count();
                    corridorRecord.weighted_width = corridor.weighted_width;
                    corridorRecord.min_sample_slack = corridor.min_sample_slack;
                    corridorRecord.anchor_clearance_radius =
                        corridor.anchor_clearance_radius;
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
                finishRecord((config.corridorMethod == "tf_sfc" ||
                              config.corridorMethod == "obb")
                                 ? "tf_sfc_generation_failure"
                                 : (config.corridorMethod == "tf_firi"
                                      ? "tf_firi_generation_failure"
                                      : (config.corridorMethod == "ellipsoid_decomp"
                                      ? "ellipsoid_decomp_generation_failure"
                                      : "invalid_corridor_method")),
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
                const gcopter::GCOPTER_PolytopeSFC::CorridorDiagnostics &initialDiagnostics =
                    gcopter.getInitialCorridorDiagnostics();
                const gcopter::GCOPTER_PolytopeSFC::CorridorDiagnostics &finalDiagnostics =
                    gcopter.getFinalCorridorDiagnostics();
                record.corridor_constrained_piece_count =
                    initialDiagnostics.constrainedPieceCount;
                record.corridor_penalty_cost_initial = initialDiagnostics.penaltyCost;
                record.corridor_penalty_cost_final = finalDiagnostics.penaltyCost;
                record.max_corridor_violation_initial_m =
                    initialDiagnostics.maxViolationM;
                record.max_corridor_violation_final_m =
                    finalDiagnostics.maxViolationM;
                record.optimizer_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - optimizerStarted)
                        .count();

                // Temporary debug tool for validating the MINCO-induced
                // deformation metric.  It only runs when the experiment tag
                // is exactly "debug_metric", so normal benchmarks are not
                // affected.
                if (config.experimentTag == "debug_metric" &&
                    std::isfinite(record.final_cost))
                {
                    gcopter::GCOPTER_PolytopeSFC::
                        GaussNewtonDeformationMetrics
                            gnMetrics;
                
                    const double gnStep =
                        0.01;
                
                    const double gnRelativeDamping =
                        1.0e-3;
                
                    const auto metricStarted =
                        std::chrono::steady_clock::now();
                
                    const bool success =
                        gcopter.computeGaussNewtonDeformationMetrics(
                            gnMetrics,
                            gnStep,
                            gnRelativeDamping);
                        
                    const double elapsedMs =
                        std::chrono::duration<
                            double,
                            std::milli>(
                                std::chrono::steady_clock::now() -
                                metricStarted)
                            .count();
                            
                    ROS_INFO_STREAM(
                        "TF_GN_COMPONENT_END "
                        << "success=" << success
                        << " pieces=" << gnMetrics.size()
                        << " elapsed_ms=" << elapsedMs);
                    
                    for (size_t pieceId = 0;
                         pieceId < gnMetrics.size();
                         ++pieceId)
                    {
                        const auto &metric =
                            gnMetrics[pieceId];
                    
                        if (!metric.valid)
                        {
                            ROS_WARN_STREAM(
                                "TF_GN_COMPONENT "
                                << "piece=" << pieceId
                                << " INVALID reason="
                                << metric.failureReason);
                            
                            continue;
                        }
                    
                        const double fractionSum =
                            metric.velocityTraceFraction +
                            metric.bodyRateTraceFraction +
                            metric.tiltTraceFraction +
                            metric.thrustTraceFraction;
                    
                        ROS_INFO_STREAM(
                            "TF_GN_COMPONENT "
                            << "piece=" << pieceId
                        
                            << " frac_v="
                            << metric.velocityTraceFraction
                        
                            << " frac_omega="
                            << metric.bodyRateTraceFraction
                        
                            << " frac_tilt="
                            << metric.tiltTraceFraction
                        
                            << " frac_thrust="
                            << metric.thrustTraceFraction
                        
                            << " frac_sum="
                            << fractionSum
                        
                            << " dir_v="
                            << metric.velocityDirectionality
                        
                            << " dir_omega="
                            << metric.bodyRateDirectionality
                        
                            << " dir_tilt="
                            << metric.tiltDirectionality
                        
                            << " dir_thrust="
                            << metric.thrustDirectionality
                        
                            << " decomp_err="
                            << metric.decompositionRelativeError
                        
                            << " total_anis="
                            << metric.anisotropy);
                    }
                }

                if (!std::isfinite(record.final_cost))
                {
                    record.corridor_penalty_cost_final =
                        std::numeric_limits<double>::quiet_NaN();
                    record.max_corridor_violation_final_m =
                        std::numeric_limits<double>::quiet_NaN();
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
        if (config.fixedStartGoalEnabled)
        {
            return;
        }
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
