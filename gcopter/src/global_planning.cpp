#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/minco_support.hpp"
#include "gcopter/minco_piece_corridor.hpp"
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

                // ============================================================
                // Exact continuous-time certificate of the nominal MINCO
                // trajectory against the ORIGINAL baseline SFC.
                //
                // Current production setup uses lengthPerPiece = INFINITY,
                // hence one baseline corridor corresponds to one MINCO piece.
                //
                // This check is deliberately performed BEFORE CSGN and before
                // any trajectory-relevant corridor reconstruction.
                // ============================================================
                const auto nominalCertStarted =
                    std::chrono::steady_clock::now();

                constexpr double nominalContainmentToleranceM =
                    1.0e-6;

                bool nominalMappingValid =
                    traj.getPieceNum() ==
                    static_cast<int>(
                        hPolys.size());

                bool nominalCertificateValid =
                    nominalMappingValid &&
                    traj.getPieceNum() > 0;

                bool nominalContained =
                    nominalCertificateValid;

                int nominalFacesChecked =
                    0;

                int nominalWorstPiece =
                    -1;

                int nominalWorstFace =
                    -1;

                double nominalMaxSignedViolationM =
                    -std::numeric_limits<double>::
                        infinity();

                double nominalMinMarginM =
                    std::numeric_limits<double>::
                        infinity();

                double nominalWorstNormalizedTime =
                    0.0;

                double nominalWorstPhysicalTime =
                    0.0;

                if (nominalCertificateValid)
                {
                    for (int pieceId = 0;
                         pieceId < traj.getPieceNum();
                         ++pieceId)
                    {
                        const auto certificate =
                            traj_relevant::
                                certifyMincoPieceInPolytope(
                                    traj[pieceId],
                                    hPolys[pieceId],
                                    nominalContainmentToleranceM,
                                    1.0e-10);

                        nominalFacesChecked +=
                            certificate.checked_face_count;

                        if (!certificate.valid)
                        {
                            nominalCertificateValid =
                                false;

                            nominalContained =
                                false;

                            nominalWorstPiece =
                                pieceId;

                            break;
                        }

                        if (!certificate.contained)
                        {
                            nominalContained =
                                false;
                        }

                        if (certificate.max_signed_violation_m >
                            nominalMaxSignedViolationM)
                        {
                            nominalMaxSignedViolationM =
                                certificate
                                    .max_signed_violation_m;

                            nominalWorstPiece =
                                pieceId;

                            nominalWorstFace =
                                certificate.worst_face;

                            nominalWorstNormalizedTime =
                                certificate
                                    .worst_normalized_time;

                            nominalWorstPhysicalTime =
                                certificate
                                    .worst_physical_time;
                        }

                        nominalMinMarginM =
                            std::min(
                                nominalMinMarginM,
                                certificate.min_margin_m);
                    }
                }

                const double nominalCertMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::steady_clock::now() -
                            nominalCertStarted)
                        .count();

                const bool nominalCertSuccess =
                    nominalCertificateValid &&
                    nominalContained;

                ROS_INFO_STREAM(
                    "TF_NOMINAL_CERT "
                    << "success="
                    << nominalCertSuccess

                    << " mapping_valid="
                    << nominalMappingValid

                    << " certificate_valid="
                    << nominalCertificateValid

                    << " contained="
                    << nominalContained

                    << " traj_pieces="
                    << traj.getPieceNum()

                    << " corridors="
                    << hPolys.size()

                    << " faces_checked="
                    << nominalFacesChecked

                    << " max_signed_violation_m="
                    << nominalMaxSignedViolationM

                    << " min_margin_m="
                    << nominalMinMarginM

                    << " worst_piece="
                    << nominalWorstPiece

                    << " worst_face="
                    << nominalWorstFace

                    << " worst_tau="
                    << nominalWorstNormalizedTime

                    << " worst_t="
                    << nominalWorstPhysicalTime

                    << " cert_ms="
                    << nominalCertMs);

                // ------------------------------------------------------------
                // Nominal-stage timing.
                //
                // path_search_ms is common to both original and proposed
                // pipelines and is printed separately.
                //
                // nominal_after_route_ms is the extra cost required to obtain
                // the baseline FIRI + optimized nominal MINCO trajectory.
                // ------------------------------------------------------------
                const double nominalAfterRouteMs =
                    record.corridor_generation_ms +
                    record.optimizer_setup_ms +
                    record.optimizer_ms;

                ROS_INFO_STREAM(
                    "TF_NOMINAL_TIMING "
                    << "path_ms="
                    << record.path_search_ms

                    << " corridor_ms="
                    << record.corridor_generation_ms

                    << " setup_ms="
                    << record.optimizer_setup_ms

                    << " opt_ms="
                    << record.optimizer_ms

                    << " nominal_after_route_ms="
                    << nominalAfterRouteMs

                    << " cert_ms="
                    << nominalCertMs);

                // Temporary debug tool for validating the MINCO-induced
                // deformation metric.  It only runs when the experiment tag
                // is exactly "debug_metric", so normal benchmarks are not
                // affected.
                // ------------------------------------------------------------
                // Debug gate diagnostic.
                // ------------------------------------------------------------
                ROS_INFO_STREAM(
                    "TF_GN_DEBUG_GATE "
                    << "experiment_tag="
                    << config.experimentTag
                    << " final_cost="
                    << record.final_cost
                    << " finite_cost="
                    << std::isfinite(record.final_cost));
                    
                if (config.experimentTag == "debug_metric" &&
                    std::isfinite(record.final_cost))
                {
                    gcopter::GCOPTER_PolytopeSFC::
                        GaussNewtonDeformationMetrics metrics;
                
                    const double csgnStep =
                        0.01;
                
                    const double relativeDamping =
                        1.0e-3;
                
                    const double proximityPower =
                        4.0;
                
                    const double maxCorridorAnisotropy =
                        10.0;
                
                    const auto metricStarted =
                        std::chrono::steady_clock::now();
                
                    const bool success =
                        gcopter.computeGaussNewtonDeformationMetrics(
                            metrics,
                            csgnStep,
                            relativeDamping,
                            proximityPower,
                            maxCorridorAnisotropy);
                        
                    const double elapsedMs =
                        std::chrono::duration<
                            double,
                            std::milli>(
                                std::chrono::steady_clock::now() -
                                metricStarted)
                            .count();
                            
                    ROS_INFO_STREAM(
                        "TF_CSGN_COMPRESS_END "
                        << "success=" << success
                        << " pieces=" << metrics.size()
                        << " elapsed_ms=" << elapsedMs
                        << " h=" << csgnStep
                        << " gamma=" << proximityPower
                        << " kappa_max="
                        << maxCorridorAnisotropy);

                    // ============================================================
                    // Validate continuous-time MINCO directional support.
                    //
                    // Expected ordering for every tested unit normal:
                    //
                    //     dense sample <= exact support <= Bernstein hull.
                    //
                    // Dense sampling is only a lower-bound diagnostic.
                    // Bernstein is only an independent outer-bound diagnostic.
                    //
                    // The final MINCO-native corridor will use exact support.
                    // ============================================================
                    if (success &&
                        traj.getPieceNum() ==
                            static_cast<int>(
                                metrics.size()))
                    {
                        const auto supportStarted =
                            std::chrono::steady_clock::now();

                        constexpr int supportDenseSamples =
                            2001;

                        constexpr double supportCheckTolerance =
                            1.0e-7;

                        int supportTestCount =
                            0;

                        int invalidSupportCount =
                            0;

                        int denseOrderingViolationCount =
                            0;

                        int bernsteinOrderingViolationCount =
                            0;

                        int maxStationaryPointCount =
                            0;

                        double maxExactMinusDense =
                            0.0;

                        double maxBernsteinMinusExact =
                            0.0;

                        double maxDenseMinusExact =
                            0.0;

                        double maxExactMinusBernstein =
                            0.0;

                        for (int pieceId = 0;
                             pieceId < traj.getPieceNum();
                             ++pieceId)
                        {
                            const auto &piece =
                                traj[pieceId];

                            std::vector<
                                Eigen::Vector3d,
                                Eigen::aligned_allocator<
                                    Eigen::Vector3d>>
                                testDirections;

                            testDirections.reserve(
                                12);

                            // World axes, both signs.
                            for (int axis = 0;
                                 axis < 3;
                                 ++axis)
                            {
                                const Eigen::Vector3d direction =
                                    Eigen::Vector3d::Unit(
                                        axis);

                                testDirections.push_back(
                                    direction);

                                testDirections.push_back(
                                    -direction);
                            }

                            // CSGN principal directions, both signs.
                            if (metrics[pieceId].valid &&
                                metrics[pieceId]
                                    .corridorUtility
                                    .allFinite())
                            {
                                Eigen::SelfAdjointEigenSolver<
                                    Eigen::Matrix3d>
                                    utilitySolver(
                                        metrics[pieceId]
                                            .corridorUtility);

                                if (utilitySolver.info() ==
                                    Eigen::Success)
                                {
                                    for (int eigenId = 0;
                                         eigenId < 3;
                                         ++eigenId)
                                    {
                                        Eigen::Vector3d direction =
                                            utilitySolver
                                                .eigenvectors()
                                                .col(
                                                    eigenId);

                                        const double norm =
                                            direction.norm();

                                        if (norm > 1.0e-12)
                                        {
                                            direction /=
                                                norm;

                                            testDirections
                                                .push_back(
                                                    direction);

                                            testDirections
                                                .push_back(
                                                    -direction);
                                        }
                                    }
                                }
                            }

                            double pieceMaxExactMinusDense =
                                0.0;

                            double pieceMaxBernsteinMinusExact =
                                0.0;

                            int pieceTests =
                                0;

                            for (Eigen::Vector3d direction :
                                 testDirections)
                            {
                                const double directionNorm =
                                    direction.norm();

                                if (!direction.allFinite() ||
                                    directionNorm <= 1.0e-12)
                                {
                                    continue;
                                }

                                direction /=
                                    directionNorm;

                                const auto exact =
                                    traj_relevant::
                                        exactMincoDirectionalSupport(
                                            piece,
                                            direction);

                                const double dense =
                                    traj_relevant::
                                        denseDirectionalSupportLowerBound(
                                            piece,
                                            direction,
                                            supportDenseSamples);

                                const double bernstein =
                                    traj_relevant::
                                        bernsteinDirectionalSupportUpperBound(
                                            piece,
                                            direction);

                                ++supportTestCount;
                                ++pieceTests;

                                if (!exact.valid ||
                                    !std::isfinite(dense) ||
                                    !std::isfinite(bernstein))
                                {
                                    ++invalidSupportCount;
                                    continue;
                                }

                                maxStationaryPointCount =
                                    std::max(
                                        maxStationaryPointCount,
                                        exact
                                            .stationary_point_count);

                                const double exactMinusDense =
                                    exact.support -
                                    dense;

                                const double bernsteinMinusExact =
                                    bernstein -
                                    exact.support;

                                const double denseMinusExact =
                                    dense -
                                    exact.support;

                                const double exactMinusBernstein =
                                    exact.support -
                                    bernstein;

                                maxExactMinusDense =
                                    std::max(
                                        maxExactMinusDense,
                                        exactMinusDense);

                                maxBernsteinMinusExact =
                                    std::max(
                                        maxBernsteinMinusExact,
                                        bernsteinMinusExact);

                                maxDenseMinusExact =
                                    std::max(
                                        maxDenseMinusExact,
                                        denseMinusExact);

                                maxExactMinusBernstein =
                                    std::max(
                                        maxExactMinusBernstein,
                                        exactMinusBernstein);

                                pieceMaxExactMinusDense =
                                    std::max(
                                        pieceMaxExactMinusDense,
                                        exactMinusDense);

                                pieceMaxBernsteinMinusExact =
                                    std::max(
                                        pieceMaxBernsteinMinusExact,
                                        bernsteinMinusExact);

                                if (dense >
                                    exact.support +
                                        supportCheckTolerance)
                                {
                                    ++denseOrderingViolationCount;
                                }

                                if (exact.support >
                                    bernstein +
                                        supportCheckTolerance)
                                {
                                    ++bernsteinOrderingViolationCount;
                                }
                            }

                            ROS_INFO_STREAM(
                                "TF_MINCO_SUPPORT_PIECE "
                                << "piece="
                                << pieceId

                                << " tests="
                                << pieceTests

                                << " duration="
                                << piece.getDuration()

                                << " max_exact_minus_dense="
                                << pieceMaxExactMinusDense

                                << " max_bernstein_minus_exact="
                                << pieceMaxBernsteinMinusExact);
                        }

                        const double supportElapsedMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::
                                        steady_clock::now() -
                                    supportStarted)
                                .count();

                        const bool supportValidationPassed =
                            invalidSupportCount == 0 &&
                            denseOrderingViolationCount == 0 &&
                            bernsteinOrderingViolationCount == 0;

                        ROS_INFO_STREAM(
                            "TF_MINCO_SUPPORT_VALIDATE "
                            << "success="
                            << supportValidationPassed

                            << " pieces="
                            << traj.getPieceNum()

                            << " tests="
                            << supportTestCount

                            << " invalid="
                            << invalidSupportCount

                            << " dense_order_violations="
                            << denseOrderingViolationCount

                            << " bernstein_order_violations="
                            << bernsteinOrderingViolationCount

                            << " max_stationary_points="
                            << maxStationaryPointCount

                            << " max_exact_minus_dense="
                            << maxExactMinusDense

                            << " max_bernstein_minus_exact="
                            << maxBernsteinMinusExact

                            << " max_dense_minus_exact="
                            << maxDenseMinusExact

                            << " max_exact_minus_bernstein="
                            << maxExactMinusBernstein

                            << " elapsed_ms="
                            << supportElapsedMs);
                    }

                    // ============================================================
                    // Validate exact metric closest-point oracle.
                    //
                    // For each nominal MINCO piece, construct query points around
                    // the curve along CSGN eigen-directions and compare:
                    //
                    //     exact metric minimum <= dense sampled minimum.
                    //
                    // This validates the degree-9 stationary polynomial before it
                    // is used for obstacle separating-plane construction.
                    // ============================================================
                    if (success &&
                        traj.getPieceNum() ==
                            static_cast<int>(
                                metrics.size()))
                    {
                        constexpr int denseClosestSamples =
                            4001;

                        constexpr double closestTolerance =
                            1.0e-7;

                        const double queryTimes[3] =
                        {
                            0.2,
                            0.5,
                            0.8
                        };

                        const double queryOffsets[2] =
                        {
                            0.5,
                            1.5
                        };

                        int closestTests =
                            0;

                        int closestInvalid =
                            0;

                        int closestOrderingViolations =
                            0;

                        int maxClosestStationaryPoints =
                            0;

                        double maxDenseMinusExact =
                            0.0;

                        double maxExactMinusDense =
                            0.0;

                        const auto closestStarted =
                            std::chrono::steady_clock::now();

                        for (int pieceId = 0;
                             pieceId < traj.getPieceNum();
                             ++pieceId)
                        {
                            if (!metrics[pieceId].valid ||
                                !metrics[pieceId]
                                    .corridorUtility
                                    .allFinite())
                            {
                                continue;
                            }

                            Eigen::SelfAdjointEigenSolver<
                                Eigen::Matrix3d>
                                utilitySolver(
                                    metrics[pieceId]
                                        .corridorUtility);

                            if (utilitySolver.info() !=
                                Eigen::Success ||
                                utilitySolver
                                        .eigenvalues()
                                        .minCoeff() <=
                                    1.0e-12)
                            {
                                continue;
                            }

                            const Eigen::Matrix3d inverseUtility =
                                utilitySolver
                                    .eigenvectors() *
                                utilitySolver
                                    .eigenvalues()
                                    .cwiseInverse()
                                    .asDiagonal() *
                                utilitySolver
                                    .eigenvectors()
                                    .transpose();

                            const auto &piece =
                                traj[pieceId];

                            int pieceTests =
                                0;

                            double pieceMaxDenseMinusExact =
                                0.0;

                            for (const double tauQuery :
                                 queryTimes)
                            {
                                const Eigen::Vector3d basePoint =
                                    piece.getPos(
                                        tauQuery *
                                        piece.getDuration());

                                for (const double offset :
                                     queryOffsets)
                                {
                                    for (int eigenId = 0;
                                         eigenId < 3;
                                         ++eigenId)
                                    {
                                        Eigen::Vector3d direction =
                                            utilitySolver
                                                .eigenvectors()
                                                .col(
                                                    eigenId);

                                        direction.normalize();

                                        for (const double sign :
                                             {-1.0, 1.0})
                                        {
                                            const Eigen::Vector3d query =
                                                basePoint +
                                                sign *
                                                offset *
                                                direction;

                                            const auto exactClosest =
                                                traj_relevant::
                                                    exactMincoMetricClosestPoint(
                                                        piece,
                                                        query,
                                                        inverseUtility);

                                            const double denseClosest =
                                                traj_relevant::
                                                    denseMetricClosestPointUpperBound(
                                                        piece,
                                                        query,
                                                        inverseUtility,
                                                        denseClosestSamples);

                                            ++closestTests;
                                            ++pieceTests;

                                            if (!exactClosest.valid ||
                                                !std::isfinite(
                                                    denseClosest))
                                            {
                                                ++closestInvalid;
                                                continue;
                                            }

                                            maxClosestStationaryPoints =
                                                std::max(
                                                    maxClosestStationaryPoints,
                                                    exactClosest
                                                        .stationary_point_count);

                                            const double denseMinusExact =
                                                denseClosest -
                                                exactClosest
                                                    .metric_distance_squared;

                                            const double exactMinusDense =
                                                exactClosest
                                                    .metric_distance_squared -
                                                denseClosest;

                                            maxDenseMinusExact =
                                                std::max(
                                                    maxDenseMinusExact,
                                                    denseMinusExact);

                                            maxExactMinusDense =
                                                std::max(
                                                    maxExactMinusDense,
                                                    exactMinusDense);

                                            pieceMaxDenseMinusExact =
                                                std::max(
                                                    pieceMaxDenseMinusExact,
                                                    denseMinusExact);

                                            if (exactClosest
                                                    .metric_distance_squared >
                                                denseClosest +
                                                    closestTolerance)
                                            {
                                                ++closestOrderingViolations;
                                            }
                                        }
                                    }
                                }
                            }

                            ROS_INFO_STREAM(
                                "TF_MINCO_CLOSEST_PIECE "
                                << "piece="
                                << pieceId

                                << " tests="
                                << pieceTests

                                << " max_dense_minus_exact="
                                << pieceMaxDenseMinusExact);
                        }

                        const double closestElapsedMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::
                                        steady_clock::now() -
                                    closestStarted)
                                .count();

                        ROS_INFO_STREAM(
                            "TF_MINCO_CLOSEST_VALIDATE "
                            << "success="
                            << (closestInvalid == 0 &&
                                closestOrderingViolations == 0)

                            << " tests="
                            << closestTests

                            << " invalid="
                            << closestInvalid

                            << " ordering_violations="
                            << closestOrderingViolations

                            << " max_stationary_points="
                            << maxClosestStationaryPoints

                            << " max_dense_minus_exact="
                            << maxDenseMinusExact

                            << " max_exact_minus_dense="
                            << maxExactMinusDense

                            << " elapsed_ms="
                            << closestElapsedMs);
                    }

                    struct MetricAnchor
                    {
                        int pieceId = -1;
                    
                        Eigen::Vector3d position =
                            Eigen::Vector3d::Zero();
                    
                        Eigen::Matrix3d utility =
                            Eigen::Matrix3d::Identity();
                    };

                    typedef std::vector<
                        MetricAnchor,
                        Eigen::aligned_allocator<MetricAnchor>>
                        MetricAnchors;

                    MetricAnchors metricAnchors;
                    
                    if (success &&
                        traj.getPieceNum() ==
                            static_cast<int>(metrics.size()))
                    {
                        constexpr int anchorSamplesPerPiece =
                            5;
                    
                        metricAnchors.reserve(
                            metrics.size() *
                            anchorSamplesPerPiece);
                        
                        for (int pieceId = 0;
                             pieceId < traj.getPieceNum();
                             ++pieceId)
                        {
                            const auto &metric =
                                metrics[pieceId];
                        
                            if (!metric.valid)
                            {
                                continue;
                            }
                        
                            const auto &piece =
                                traj[pieceId];
                        
                            const double duration =
                                piece.getDuration();
                        
                            for (int sampleId = 0;
                                 sampleId <
                                     anchorSamplesPerPiece;
                                 ++sampleId)
                            {
                                const double alpha =
                                    static_cast<double>(sampleId) /
                                    static_cast<double>(
                                        anchorSamplesPerPiece - 1);
                                    
                                MetricAnchor anchor;
                                    
                                anchor.pieceId =
                                    pieceId;
                                    
                                anchor.position =
                                    piece.getPos(
                                        alpha *
                                        duration);
                                    
                                anchor.utility =
                                    metric.corridorUtility;
                                    
                                metricAnchors.push_back(
                                    anchor);
                            }
                        }
                    }

                    struct MappingSegment
                    {
                        Eigen::Vector3d a =
                            Eigen::Vector3d::Zero();
                    
                        Eigen::Vector3d b =
                            Eigen::Vector3d::Zero();
                    
                        int metricPieceId = -1;
                    
                        double mappingDistance =
                            INFINITY;
                    
                        Eigen::Matrix3d utility =
                            Eigen::Matrix3d::Identity();
                    };

                    typedef std::vector<
                        MappingSegment,
                        Eigen::aligned_allocator<MappingSegment>>
                        MappingSegments;

                    MappingSegments mappingSegments;

                    if (!metricAnchors.empty())
                    {
                        const double progress =
                            std::max(
                                config.tfFiriProgress,
                                config.voxelWidth);
                            
                        const int routePointCount =
                            static_cast<int>(
                                route.size());
                            
                        Eigen::Vector3d b =
                            route.front();
                            
                        for (int routeId = 1;
                             routeId < routePointCount;)
                        {
                            const Eigen::Vector3d a =
                                b;
                        
                            if ((a - route[routeId]).norm() >
                                progress)
                            {
                                b =
                                    (route[routeId] - a)
                                        .normalized() *
                                    progress +
                                    a;
                            }
                            else
                            {
                                b =
                                    route[routeId];
                            
                                ++routeId;
                            }
                        
                            MappingSegment segment;
                        
                            segment.a =
                                a;
                        
                            segment.b =
                                b;
                        
                            const Eigen::Vector3d midpoint =
                                0.5 *
                                (a + b);
                        
                            for (const auto &anchor :
                                 metricAnchors)
                            {
                                const double distance =
                                    (midpoint -
                                     anchor.position)
                                        .norm();
                                    
                                if (distance <
                                    segment.mappingDistance)
                                {
                                    segment.mappingDistance =
                                        distance;
                                
                                    segment.metricPieceId =
                                        anchor.pieceId;
                                
                                    segment.utility =
                                        anchor.utility;
                                }
                            }
                        
                            mappingSegments.push_back(
                                segment);
                        }
                    }

                    ROS_INFO_STREAM(
                        "TF_METRIC_MAP_END "
                        << "anchors="
                        << metricAnchors.size()
                        << " segments="
                        << mappingSegments.size()
                        << " minco_pieces="
                        << metrics.size());

                    for (size_t segmentId = 0;
                         segmentId < mappingSegments.size();
                         ++segmentId)
                    {
                        const auto &segment =
                            mappingSegments[segmentId];
                    
                        Eigen::SelfAdjointEigenSolver<
                            Eigen::Matrix3d>
                            utilitySolver(
                                segment.utility);
                            
                        double mappedAnisotropy =
                            INFINITY;
                            
                        if (utilitySolver.info() ==
                            Eigen::Success)
                        {
                            const Eigen::Vector3d eig =
                                utilitySolver.eigenvalues();
                        
                            if (eig.minCoeff() > 0.0)
                            {
                                mappedAnisotropy =
                                    eig.maxCoeff() /
                                    eig.minCoeff();
                            }
                        }
                    
                        ROS_INFO_STREAM(
                            "TF_METRIC_MAP "
                            << "segment="
                            << segmentId
                        
                            << " metric_piece="
                            << segment.metricPieceId
                        
                            << " distance="
                            << segment.mappingDistance
                        
                            << " anis="
                            << mappedAnisotropy
                        
                            << " midpoint=["
                            << 0.5 *
                                   (segment.a.x() +
                                    segment.b.x())
                            << ","
                            << 0.5 *
                                   (segment.a.y() +
                                    segment.b.y())
                            << ","
                            << 0.5 *
                                   (segment.a.z() +
                                    segment.b.z())
                            << "]");
                    }

                    if (!mappingSegments.empty())
                    {
                        double meanMappingDistance =
                            0.0;
                    
                        double maxMappingDistance =
                            0.0;
                    
                        int invalidMappingCount =
                            0;
                    
                        for (const auto &segment :
                             mappingSegments)
                        {
                            if (segment.metricPieceId < 0 ||
                                !std::isfinite(
                                    segment.mappingDistance))
                            {
                                ++invalidMappingCount;
                                continue;
                            }
                        
                            meanMappingDistance +=
                                segment.mappingDistance;
                        
                            maxMappingDistance =
                                std::max(
                                    maxMappingDistance,
                                    segment.mappingDistance);
                        }
                    
                        const int validMappingCount =
                            static_cast<int>(
                                mappingSegments.size()) -
                            invalidMappingCount;
                            
                        if (validMappingCount > 0)
                        {
                            meanMappingDistance /=
                                static_cast<double>(
                                    validMappingCount);
                        }
                    
                        ROS_INFO_STREAM(
                            "TF_METRIC_MAP_SUMMARY "
                            << "valid="
                            << validMappingCount
                            << "/"
                            << mappingSegments.size()
                            << " mean_distance="
                            << meanMappingDistance
                            << " max_distance="
                            << maxMappingDistance);

                        // ============================================================
                        // Convert the validated spatial mapping into the metric format
                        // consumed by the second-pass trajectory-conditioned FIRI.
                        //
                        // IMPORTANT:
                        // This does NOT modify the first-pass hPolys used by the
                        // current GCOPTER optimization.  It only prepares a second-pass
                        // diagnostic FIRI run.
                        // ============================================================
                        sfc_gen::SegmentDeformationMetrics
                            mappedFiriMetrics;

                        mappedFiriMetrics.reserve(
                            mappingSegments.size());
                        
                        // A mapped MINCO metric is considered local enough only when
                        // its nearest trajectory anchor is inside the same spatial
                        // scale used to collect local FIRI obstacles.
                        const double metricMappingDistanceLimit =
                            std::max(
                                config.tfFiriRange,
                                config.voxelWidth);
                            
                        int validMappedMetricCount =
                            0;
                            
                        for (const auto &segment :
                             mappingSegments)
                        {
                            sfc_gen::SegmentDeformationMetric
                                mappedMetric;
                        
                            mappedMetric.source_piece_id =
                                segment.metricPieceId;
                        
                            mappedMetric.mapping_distance =
                                segment.mappingDistance;
                        
                            mappedMetric.utility =
                                segment.utility;
                        
                            mappedMetric.valid =
                                segment.metricPieceId >= 0 &&
                                std::isfinite(
                                    segment.mappingDistance) &&
                                segment.mappingDistance <=
                                    metricMappingDistanceLimit &&
                                segment.utility.allFinite();
                                
                            if (mappedMetric.valid)
                            {
                                ++validMappedMetricCount;
                            }
                        
                            mappedFiriMetrics.push_back(
                                mappedMetric);
                        }

                        ROS_INFO_STREAM(
                            "TF_FIRI_METRIC_INPUT "
                            << "segments="
                            << mappedFiriMetrics.size()
                            << " valid="
                            << validMappedMetricCount
                            << " distance_limit="
                            << metricMappingDistanceLimit);

                        // ============================================================
                        // A/B options.
                        //
                        // CONTROL:
                        //   same face budget,
                        //   same coverage/face-count term,
                        //   same candidate pool,
                        //   native MVIE,
                        //   NO route-direction bias,
                        //   NO CSGN metric term.
                        //
                        // METRIC:
                        //   exactly the same settings,
                        //   except metric_weight = 1.
                        // ============================================================
                        firi::TrajectoryFavorableOptions
                            controlOptions;
                                                
                        controlOptions.enabled =
                            true;
                                                
                        // Disable the old route-direction preference.
                        // We want this experiment to isolate CSGN.
                        controlOptions.directional_width_weight =
                            0.0;
                                                
                        controlOptions.face_count_weight =
                            std::max(
                                0.0,
                                config.tfFiriFaceCountWeight);
                            
                        controlOptions.candidate_pool_size =
                            std::max(
                                1,
                                config.tfFiriCandidatePoolSize);
                            
                        // Pure CSGN geometry-response experiment:
                        //
                        // Disable the HARD face-count cap so that both CONTROL and
                        // METRIC are allowed to finish constructing obstacle-free
                        // corridors.
                        //
                        // The soft face-count/coverage score remains active, and both
                        // sides still use the same candidate pool.  Therefore the only
                        // A/B difference remains metric_weight = 0 vs 1.
                        controlOptions.max_faces =
                            0;
                            
                        // The per-segment metric itself will still be supplied,
                        // but weight zero means it has no effect on candidate score.
                        controlOptions.metric_weight =
                            0.0;
                            
                        firi::TrajectoryFavorableOptions
                            metricOptions =
                                controlOptions;
                            
                        metricOptions.metric_weight =
                            1.0;

                        // ============================================================
                        // Separate containers so the second-pass experiment cannot
                        // overwrite the original hPolys used by the nominal GCOPTER.
                        // ============================================================
                        std::vector<Eigen::MatrixX4d>
                            controlSecondPassHPolys;
                                                    
                        std::vector<
                            sfc_gen::TrajectoryFavorableFiriInfo>
                            controlSecondPassInfos;
                                                    
                        std::vector<Eigen::MatrixX4d>
                            metricSecondPassHPolys;
                                                    
                        std::vector<
                            sfc_gen::TrajectoryFavorableFiriInfo>
                            metricSecondPassInfos;
                        
                        // ============================================================
                        // CONTROL second pass.
                        // ============================================================
                        const auto controlFiriStarted =
                            std::chrono::steady_clock::now();

                        const bool controlFiriSuccess =
                            sfc_gen::trajectoryFavorableConvexCover(
                                route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                std::max(
                                    config.tfFiriProgress,
                                    config.voxelWidth),
                                std::max(
                                    config.tfFiriRange,
                                    config.voxelWidth),
                                controlOptions,
                                controlSecondPassHPolys,
                                controlSecondPassInfos,
                                &mappedFiriMetrics);
                                
                        const double controlFiriMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::steady_clock::now() -
                                    controlFiriStarted)
                                .count();
                    
                        // ============================================================
                        // CSGN metric-aware second pass.
                        // ============================================================
                        const auto metricFiriStarted =
                            std::chrono::steady_clock::now();

                        const bool metricFiriSuccess =
                            sfc_gen::trajectoryFavorableConvexCover(
                                route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                std::max(
                                    config.tfFiriProgress,
                                    config.voxelWidth),
                                std::max(
                                    config.tfFiriRange,
                                    config.voxelWidth),
                                metricOptions,
                                metricSecondPassHPolys,
                                metricSecondPassInfos,
                                &mappedFiriMetrics);
                                
                        const double metricFiriMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::steady_clock::now() -
                                    metricFiriStarted)
                                .count();

                        auto countSecondPassFaces =
                            [](const std::vector<
                                   Eigen::MatrixX4d> &polys)
                                -> int
                        {
                            int totalFaceCount =
                                0;
                        
                            for (const auto &poly :
                                 polys)
                            {
                                totalFaceCount +=
                                    static_cast<int>(
                                        poly.rows());
                            }
                        
                            return totalFaceCount;
                        };
                        const int controlFaceCount =
                            countSecondPassFaces(
                                controlSecondPassHPolys);
                            
                        const int metricFaceCount =
                            countSecondPassFaces(
                                metricSecondPassHPolys);
                        int changedCorridorCount =
                            0;

                        const std::size_t comparableCorridorCount =
                            std::min(
                                controlSecondPassHPolys.size(),
                                metricSecondPassHPolys.size());
                            
                        for (std::size_t corridorId = 0;
                             corridorId <
                                 comparableCorridorCount;
                             ++corridorId)
                        {
                            const auto &controlPoly =
                                controlSecondPassHPolys[
                                    corridorId];
                                
                            const auto &metricPoly =
                                metricSecondPassHPolys[
                                    corridorId];
                                
                            bool corridorChanged =
                                false;
                                
                            if (controlPoly.rows() !=
                                    metricPoly.rows() ||
                                controlPoly.cols() !=
                                    metricPoly.cols())
                            {
                                corridorChanged =
                                    true;
                            }
                            else
                            {
                                const double relativeDifference =
                                    (controlPoly -
                                     metricPoly)
                                        .norm() /
                                    std::max(
                                        controlPoly.norm(),
                                        1.0e-12);
                                    
                                corridorChanged =
                                    relativeDifference >
                                    1.0e-8;
                                    
                                ROS_INFO_STREAM(
                                    "TF_CSGN_FIRI_DIFF "
                                    << "corridor="
                                    << corridorId
                                    << " rel_diff="
                                    << relativeDifference
                                    << " control_faces="
                                    << controlPoly.rows()
                                    << " metric_faces="
                                    << metricPoly.rows());
                            }
                        
                            if (corridorChanged)
                            {
                                ++changedCorridorCount;
                            }
                        }

                        // A different final corridor count is itself a geometric
                        // change, so include unmatched corridors.
                        changedCorridorCount +=
                            static_cast<int>(
                                std::max(
                                    controlSecondPassHPolys.size(),
                                    metricSecondPassHPolys.size()) -
                                comparableCorridorCount);

                        // ============================================================
                        // Diagnose second-pass FIRI failures.
                        //
                        // If trajectoryFavorableConvexCover() fails inside the first
                        // raw segment, hpolys is still empty but infos contains the
                        // diagnostics produced by that failed FIRI call.
                        // ============================================================
                        if (!controlFiriSuccess)
                        {
                            if (!controlSecondPassInfos.empty())
                            {
                                const auto &failure =
                                    controlSecondPassInfos.back();
                            
                                ROS_WARN_STREAM(
                                    "TF_CSGN_FIRI_FAILURE "
                                    << "mode=control"
                                
                                    << " info_count="
                                    << controlSecondPassInfos.size()
                                
                                    << " completed_corridors="
                                    << controlSecondPassHPolys.size()
                                
                                    << " face_count="
                                    << failure.face_count
                                
                                    << " budget_saturated="
                                    << failure.face_budget_saturated
                                
                                    << " unresolved_total="
                                    << failure.unresolved_constraint_count
                                
                                    << " unresolved_boundary="
                                    << failure.unresolved_boundary_count
                                
                                    << " unresolved_obstacle="
                                    << failure.unresolved_obstacle_count
                                
                                    << " exchange_attempted="
                                    << failure.budget_exchange_attempted
                                
                                    << " exchange_accepted="
                                    << failure.budget_exchange_accepted);
                            }
                            else
                            {
                                ROS_WARN(
                                    "TF_CSGN_FIRI_FAILURE "
                                    "mode=control no_failure_info");
                            }
                        }

                        if (!metricFiriSuccess)
                        {
                            if (!metricSecondPassInfos.empty())
                            {
                                const auto &failure =
                                    metricSecondPassInfos.back();
                            
                                ROS_WARN_STREAM(
                                    "TF_CSGN_FIRI_FAILURE "
                                    << "mode=metric"
                                
                                    << " info_count="
                                    << metricSecondPassInfos.size()
                                
                                    << " completed_corridors="
                                    << metricSecondPassHPolys.size()
                                
                                    << " face_count="
                                    << failure.face_count
                                
                                    << " budget_saturated="
                                    << failure.face_budget_saturated
                                
                                    << " unresolved_total="
                                    << failure.unresolved_constraint_count
                                
                                    << " unresolved_boundary="
                                    << failure.unresolved_boundary_count
                                
                                    << " unresolved_obstacle="
                                    << failure.unresolved_obstacle_count
                                
                                    << " exchange_attempted="
                                    << failure.budget_exchange_attempted
                                
                                    << " exchange_accepted="
                                    << failure.budget_exchange_accepted);
                            }
                            else
                            {
                                ROS_WARN(
                                    "TF_CSGN_FIRI_FAILURE "
                                    "mode=metric no_failure_info");
                            }
                        }

                        // ============================================================
                        // Compare the final selected obstacle faces under the same
                        // compressed CSGN utility.
                        //
                        // Lower mean_metric_damage is better: it means the corridor
                        // places fewer obstacle-face normals along MINCO's easy
                        // deformation directions.
                        // ============================================================
                        const std::size_t comparableInfoCount =
                            std::min(
                                controlSecondPassInfos.size(),
                                metricSecondPassInfos.size());

                        for (std::size_t corridorId = 0;
                             corridorId <
                                 comparableInfoCount;
                             ++corridorId)
                        {
                            const auto &controlInfo =
                                controlSecondPassInfos[
                                    corridorId];

                            const auto &metricInfo =
                                metricSecondPassInfos[
                                    corridorId];

                            ROS_INFO_STREAM(
                                "TF_CSGN_FIRI_DAMAGE "
                                << "corridor="
                                << corridorId

                                << " control_obs_faces="
                                << controlInfo.obstacle_face_count

                                << " metric_obs_faces="
                                << metricInfo.obstacle_face_count

                                << " control_mean_psi="
                                << controlInfo.mean_metric_damage

                                << " metric_mean_psi="
                                << metricInfo.mean_metric_damage

                                << " delta_mean_psi="
                                << (metricInfo.mean_metric_damage -
                                    controlInfo.mean_metric_damage)

                                << " control_min_psi="
                                << controlInfo.min_metric_damage

                                << " metric_min_psi="
                                << metricInfo.min_metric_damage

                                << " control_max_psi="
                                << controlInfo.max_metric_damage

                                << " metric_max_psi="
                                << metricInfo.max_metric_damage);
                        }

                        int controlObstacleFaceCount =
                            0;

                        int metricObstacleFaceCount =
                            0;

                        double controlWeightedDamageSum =
                            0.0;

                        double metricWeightedDamageSum =
                            0.0;

                        for (const auto &info :
                             controlSecondPassInfos)
                        {
                            controlObstacleFaceCount +=
                                info.obstacle_face_count;

                            controlWeightedDamageSum +=
                                static_cast<double>(
                                    info.obstacle_face_count) *
                                info.mean_metric_damage;
                        }

                        for (const auto &info :
                             metricSecondPassInfos)
                        {
                            metricObstacleFaceCount +=
                                info.obstacle_face_count;

                            metricWeightedDamageSum +=
                                static_cast<double>(
                                    info.obstacle_face_count) *
                                info.mean_metric_damage;
                        }

                        const double controlMeanMetricDamage =
                            controlObstacleFaceCount > 0
                                ? controlWeightedDamageSum /
                                      static_cast<double>(
                                          controlObstacleFaceCount)
                                : 0.0;

                        const double metricMeanMetricDamage =
                            metricObstacleFaceCount > 0
                                ? metricWeightedDamageSum /
                                      static_cast<double>(
                                          metricObstacleFaceCount)
                                : 0.0;

                        ROS_INFO_STREAM(
                            "TF_CSGN_FIRI_DAMAGE_SUMMARY "
                            << "control_obs_faces="
                            << controlObstacleFaceCount

                            << " metric_obs_faces="
                            << metricObstacleFaceCount

                            << " control_mean_psi="
                            << controlMeanMetricDamage

                            << " metric_mean_psi="
                            << metricMeanMetricDamage

                            << " delta_mean_psi="
                            << (metricMeanMetricDamage -
                                controlMeanMetricDamage));

                        ROS_INFO_STREAM(
                            "TF_CSGN_FIRI_AB "
                            << "control_success="
                            << controlFiriSuccess
                            << " metric_success="
                            << metricFiriSuccess
                        
                            << " mapped_metrics="
                            << mappedFiriMetrics.size()
                            << " mapped_valid="
                            << validMappedMetricCount
                        
                            << " control_corridors="
                            << controlSecondPassHPolys.size()
                            << " metric_corridors="
                            << metricSecondPassHPolys.size()

                            << " control_infos="
                            << controlSecondPassInfos.size()
                                                    
                            << " metric_infos="
                            << metricSecondPassInfos.size()
                        
                            << " control_faces="
                            << controlFaceCount
                            << " metric_faces="
                            << metricFaceCount
                        
                            << " changed_corridors="
                            << changedCorridorCount
                        
                            << " control_ms="
                            << controlFiriMs
                            << " metric_ms="
                            << metricFiriMs);

                        // ============================================================
                        // Trajectory-relevant minimum-face corridor experiment.
                        //
                        // This is the first test of the NEW construction algorithm.
                        //
                        // It does NOT use:
                        //   - FIRI MVIE,
                        //   - FIRI inflation iterations,
                        //   - face_count_weight,
                        //   - candidate_pool_size,
                        //   - hard FIRI face budget.
                        //
                        // Instead:
                        //
                        //   nominal MINCO
                        //       -> mapped CSGN utility
                        //       -> anisotropic trajectory-relevant domain
                        //       -> batch separating-plane set cover
                        //       -> redundancy pruning.
                        //
                        // max_extra_radius uses the same spatial scale as the
                        // existing FIRI local range.
                        // ============================================================
                        traj_relevant::CompactCorridorOptions
                            compactOptions;

                        compactOptions.max_extra_radius =
                            std::max(
                                config.tfFiriRange,
                                config.voxelWidth);

                        // MVP hypothesis:
                        //
                        // low-value MINCO directions receive only 25% of the
                        // maximum extra expansion.
                        compactOptions.min_extra_ratio =
                            0.25;

                        // Explicit finite-radius junction/segment protection.
                        compactOptions.overlap_radius =
                            0.01;

                        compactOptions.epsilon =
                            1.0e-6;

                        std::vector<Eigen::MatrixX4d>
                            compactHPolys;

                        sfc_gen::TrajectoryRelevantCompactInfos
                            compactInfos;

                        const auto compactStarted =
                            std::chrono::steady_clock::now();

                        const bool compactSuccess =
                            sfc_gen::
                                trajectoryRelevantCompactCover(
                                    route,
                                    pc,
                                    voxelMap.getOrigin(),
                                    voxelMap.getCorner(),
                                    std::max(
                                        config.tfFiriProgress,
                                        config.voxelWidth),
                                    compactOptions,
                                    compactHPolys,
                                    compactInfos,
                                    &mappedFiriMetrics);

                        const double compactMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::steady_clock::now() -
                                    compactStarted)
                                .count();

                        const int compactTotalFaces =
                            countSecondPassFaces(
                                compactHPolys);

                        // The currently optimized nominal corridor is native
                        // FIRI in the fixed debug experiment.
                        const int nominalFiriTotalFaces =
                            countSecondPassFaces(
                                hPolys);

                        int compactDomainFaces =
                            0;

                        int compactObstacleFaces =
                            0;

                        int compactCandidates =
                            0;

                        int compactGreedyFaces =
                            0;

                        int compactRedundancyRemoved =
                            0;

                        int compactMetricValidCount =
                            0;

                        int compactAnisotropicCount =
                            0;

                        int compactSafetyVerifiedCount =
                            0;

                        int compactOverlapGuaranteedCount =
                            0;

                        double compactWeightedDamageSum =
                            0.0;

                        for (const auto &info :
                             compactInfos)
                        {
                            compactDomainFaces +=
                                info.domain_face_count;

                            compactObstacleFaces +=
                                info.selected_obstacle_face_count;

                            compactCandidates +=
                                info.candidate_count;

                            compactGreedyFaces +=
                                info.greedy_obstacle_face_count;

                            compactRedundancyRemoved +=
                                info.redundancy_removed;

                            compactMetricValidCount +=
                                info.metric_valid
                                    ? 1
                                    : 0;

                            compactAnisotropicCount +=
                                info.anisotropic_domain
                                    ? 1
                                    : 0;

                            compactSafetyVerifiedCount +=
                                info.safety_verified
                                    ? 1
                                    : 0;

                            compactOverlapGuaranteedCount +=
                                info.overlap_guaranteed
                                    ? 1
                                    : 0;

                            compactWeightedDamageSum +=
                                static_cast<double>(
                                    info.selected_obstacle_face_count) *
                                info.mean_metric_damage;
                        }

                        const double compactMeanMetricDamage =
                            compactObstacleFaces > 0
                                ? compactWeightedDamageSum /
                                      static_cast<double>(
                                          compactObstacleFaces)
                                : 0.0;

                        ROS_INFO_STREAM(
                            "TF_MINFACE_AB "
                            << "success="
                            << compactSuccess

                            << " mapped_metrics="
                            << mappedFiriMetrics.size()

                            << " mapped_valid="
                            << validMappedMetricCount

                            << " nominal_firi_corridors="
                            << hPolys.size()

                            << " control_firi_corridors="
                            << controlSecondPassHPolys.size()

                            << " compact_corridors="
                            << compactHPolys.size()

                            << " nominal_firi_faces="
                            << nominalFiriTotalFaces

                            << " control_firi_faces="
                            << controlFaceCount

                            << " compact_faces="
                            << compactTotalFaces

                            << " control_firi_obs_faces="
                            << controlObstacleFaceCount

                            << " compact_obs_faces="
                            << compactObstacleFaces

                            << " compact_domain_faces="
                            << compactDomainFaces

                            << " compact_candidates="
                            << compactCandidates

                            << " compact_greedy_faces="
                            << compactGreedyFaces

                            << " compact_redundancy_removed="
                            << compactRedundancyRemoved

                            << " control_mean_psi="
                            << controlMeanMetricDamage

                            << " compact_mean_psi="
                            << compactMeanMetricDamage

                            << " compact_metric_valid="
                            << compactMetricValidCount
                            << "/"
                            << compactInfos.size()

                            << " compact_anisotropic="
                            << compactAnisotropicCount
                            << "/"
                            << compactInfos.size()

                            << " compact_safety="
                            << compactSafetyVerifiedCount
                            << "/"
                            << compactInfos.size()

                            << " compact_overlap="
                            << compactOverlapGuaranteedCount
                            << "/"
                            << compactInfos.size()

                            << " compact_ms="
                            << compactMs);

                        // ============================================================
                        // Per-polytope face-budget sweep.
                        //
                        // Goal:
                        //   Measure feasibility, final face complexity, CSGN face
                        //   damage, and FIRI runtime under the same hard face budget.
                        //
                        // IMPORTANT:
                        //   max_faces is a PER-FIRI-REGION cap, not a global corridor
                        //   face-count cap.
                        //
                        // CONTROL and METRIC use identical settings except:
                        //   CONTROL: metric_weight = 0
                        //   METRIC : metric_weight = 1
                        //
                        // Native MVIE remains unchanged.
                        // ============================================================
                        struct FaceBudgetRunSummary
                        {
                            bool success =
                                false;

                            int budget =
                                0;

                            int corridorCount =
                                0;

                            int infoCount =
                                0;

                            int totalFaces =
                                0;

                            int obstacleFaces =
                                0;

                            double meanMetricDamage =
                                std::numeric_limits<double>::quiet_NaN();

                            double elapsedMs =
                                0.0;

                            // Valid when success == false and failure info exists.
                            int failureFaceCount =
                                0;

                            bool failureBudgetSaturated =
                                false;

                            int unresolvedTotal =
                                0;

                            int unresolvedBoundary =
                                0;

                            int unresolvedObstacle =
                                0;

                            bool exchangeAttempted =
                                false;

                            bool exchangeAccepted =
                                false;
                        };

                        auto runFaceBudgetCase =
                            [&](const int faceBudget,
                                const double metricWeight)
                                -> FaceBudgetRunSummary
                        {
                            FaceBudgetRunSummary summary;

                            summary.budget =
                                faceBudget;

                            // Start from exactly the same validated CONTROL settings.
                            firi::TrajectoryFavorableOptions
                                sweepOptions =
                                    controlOptions;

                            // Activate the hard PER-REGION face budget.
                            sweepOptions.max_faces =
                                faceBudget;

                            // This is the only CONTROL/METRIC difference.
                            sweepOptions.metric_weight =
                                metricWeight;

                            std::vector<Eigen::MatrixX4d>
                                sweepHPolys;

                            std::vector<
                                sfc_gen::TrajectoryFavorableFiriInfo>
                                sweepInfos;

                            const auto sweepStarted =
                                std::chrono::steady_clock::now();

                            summary.success =
                                sfc_gen::trajectoryFavorableConvexCover(
                                    route,
                                    pc,
                                    voxelMap.getOrigin(),
                                    voxelMap.getCorner(),
                                    std::max(
                                        config.tfFiriProgress,
                                        config.voxelWidth),
                                    std::max(
                                        config.tfFiriRange,
                                        config.voxelWidth),
                                    sweepOptions,
                                    sweepHPolys,
                                    sweepInfos,
                                    &mappedFiriMetrics);

                            summary.elapsedMs =
                                std::chrono::duration<
                                    double,
                                    std::milli>(
                                        std::chrono::steady_clock::now() -
                                        sweepStarted)
                                    .count();

                            summary.corridorCount =
                                static_cast<int>(
                                    sweepHPolys.size());

                            summary.infoCount =
                                static_cast<int>(
                                    sweepInfos.size());

                            summary.totalFaces =
                                countSecondPassFaces(
                                    sweepHPolys);

                            // --------------------------------------------------------
                            // Successful complete corridor:
                            // compute face-count-weighted mean CSGN damage.
                            // --------------------------------------------------------
                            if (summary.success)
                            {
                                double weightedDamageSum =
                                    0.0;

                                for (const auto &info :
                                     sweepInfos)
                                {
                                    summary.obstacleFaces +=
                                        info.obstacle_face_count;

                                    weightedDamageSum +=
                                        static_cast<double>(
                                            info.obstacle_face_count) *
                                        info.mean_metric_damage;
                                }

                                if (summary.obstacleFaces > 0)
                                {
                                    summary.meanMetricDamage =
                                        weightedDamageSum /
                                        static_cast<double>(
                                            summary.obstacleFaces);
                                }
                            }
                            // --------------------------------------------------------
                            // Failed corridor:
                            // the last info entry carries the failure diagnostics.
                            // --------------------------------------------------------
                            else if (!sweepInfos.empty())
                            {
                                const auto &failure =
                                    sweepInfos.back();

                                summary.failureFaceCount =
                                    failure.face_count;

                                summary.failureBudgetSaturated =
                                    failure.face_budget_saturated;

                                summary.unresolvedTotal =
                                    failure.unresolved_constraint_count;

                                summary.unresolvedBoundary =
                                    failure.unresolved_boundary_count;

                                summary.unresolvedObstacle =
                                    failure.unresolved_obstacle_count;

                                summary.exchangeAttempted =
                                    failure.budget_exchange_attempted;

                                summary.exchangeAccepted =
                                    failure.budget_exchange_accepted;
                            }

                            return summary;
                        };

                        // ============================================================
                        // Paired GCOPTER backend experiment.
                        //
                        // B = 28 is intentionally selected because, in the current
                        // deterministic case:
                        //
                        //   CONTROL total faces = 171
                        //   CSGN    total faces = 171
                        //
                        // Thus this test isolates corridor GEOMETRY from total
                        // constraint count as much as possible.
                        //
                        // Both corridors are generated from the same route and the
                        // same nominal-trajectory CSGN metrics.
                        // ============================================================
                        const int backendAbFaceBudget =
                            28;

                        std::vector<Eigen::MatrixX4d>
                            backendControlHPolys;

                        std::vector<
                            sfc_gen::TrajectoryFavorableFiriInfo>
                            backendControlInfos;

                        std::vector<Eigen::MatrixX4d>
                            backendMetricHPolys;

                        std::vector<
                            sfc_gen::TrajectoryFavorableFiriInfo>
                            backendMetricInfos;

                        firi::TrajectoryFavorableOptions
                            backendControlOptions =
                                controlOptions;

                        backendControlOptions.max_faces =
                            backendAbFaceBudget;

                        backendControlOptions.metric_weight =
                            0.0;

                        firi::TrajectoryFavorableOptions
                            backendMetricOptions =
                                backendControlOptions;

                        backendMetricOptions.metric_weight =
                            1.0;

                        const bool backendControlCorridorSuccess =
                            sfc_gen::trajectoryFavorableConvexCover(
                                route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                std::max(
                                    config.tfFiriProgress,
                                    config.voxelWidth),
                                std::max(
                                    config.tfFiriRange,
                                    config.voxelWidth),
                                backendControlOptions,
                                backendControlHPolys,
                                backendControlInfos,
                                &mappedFiriMetrics);
                                
                        const bool backendMetricCorridorSuccess =
                            sfc_gen::trajectoryFavorableConvexCover(
                                route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                std::max(
                                    config.tfFiriProgress,
                                    config.voxelWidth),
                                std::max(
                                    config.tfFiriRange,
                                    config.voxelWidth),
                                backendMetricOptions,
                                backendMetricHPolys,
                                backendMetricInfos,
                                &mappedFiriMetrics);

                        struct BackendAbResult
                        {
                            bool setup_success =
                                false;
                        
                            bool optimize_success =
                                false;
                        
                            int corridor_count =
                                0;
                        
                            int total_faces =
                                0;
                        
                            int trajectory_pieces =
                                0;
                        
                            int constrained_pieces =
                                0;
                        
                            double setup_ms =
                                0.0;
                        
                            double optimize_ms =
                                0.0;
                        
                            double final_cost =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double trajectory_duration =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double corridor_penalty_initial =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double corridor_penalty_final =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double max_corridor_violation_initial =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double max_corridor_violation_final =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                            double corridor_slack_initial =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            double corridor_slack_final =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        };

                        auto runBackendAb =
                            [&](const std::vector<
                                    Eigen::MatrixX4d> &corridors)
                                -> BackendAbResult
                        {
                            BackendAbResult result;
                        
                            result.corridor_count =
                                static_cast<int>(
                                    corridors.size());
                                
                            for (const auto &poly :
                                 corridors)
                            {
                                result.total_faces +=
                                    static_cast<int>(
                                        poly.rows());
                            }
                        
                            if (corridors.empty())
                            {
                                return result;
                            }
                        
                            gcopter::GCOPTER_PolytopeSFC
                                backendOptimizer;
                        
                            Trajectory<5>
                                backendTrajectory;
                        
                            const auto setupStarted =
                                std::chrono::steady_clock::now();
                        
                            result.setup_success =
                                backendOptimizer.setup(
                                    config.weightT,
                                    iniState,
                                    finState,
                                    corridors,
                                    INFINITY,
                                    config.smoothingEps,
                                    quadratureRes,
                                    magnitudeBounds,
                                    penaltyWeights,
                                    physicalParams);
                                
                            result.setup_ms =
                                std::chrono::duration<
                                    double,
                                    std::milli>(
                                        std::chrono::steady_clock::now() -
                                        setupStarted)
                                    .count();
                                    
                            if (!result.setup_success)
                            {
                                return result;
                            }
                        
                            const auto optimizeStarted =
                                std::chrono::steady_clock::now();
                        
                            result.final_cost =
                                backendOptimizer.optimize(
                                    backendTrajectory,
                                    config.relCostTol);
                                
                            result.optimize_ms =
                                std::chrono::duration<
                                    double,
                                    std::milli>(
                                        std::chrono::steady_clock::now() -
                                        optimizeStarted)
                                    .count();
                                    
                            const auto &initialDiagnostics =
                                backendOptimizer
                                    .getInitialCorridorDiagnostics();
                                    
                            const auto &finalDiagnostics =
                                backendOptimizer
                                    .getFinalCorridorDiagnostics();
                                    
                            result.constrained_pieces =
                                initialDiagnostics
                                    .constrainedPieceCount;
                                    
                            result.corridor_penalty_initial =
                                initialDiagnostics.penaltyCost;
                                    
                            result.corridor_penalty_final =
                                finalDiagnostics.penaltyCost;
                                    
                            result.max_corridor_violation_initial =
                                initialDiagnostics.maxViolationM;
                                    
                            result.max_corridor_violation_final =
                                finalDiagnostics.maxViolationM;

                            result.corridor_slack_initial =
                                initialDiagnostics.minSlackM;

                            result.corridor_slack_final =
                                finalDiagnostics.minSlackM;
                                    
                            if (std::isfinite(
                                    result.final_cost) &&
                                backendTrajectory.getPieceNum() > 0)
                            {
                                result.optimize_success =
                                    true;
                            
                                result.trajectory_pieces =
                                    backendTrajectory.getPieceNum();
                            
                                result.trajectory_duration =
                                    backendTrajectory
                                        .getTotalDuration();
                            }
                        
                            return result;
                        };

                        BackendAbResult
                            backendControlResult;

                        BackendAbResult
                            backendMetricResult;

                        if (backendControlCorridorSuccess)
                        {
                            backendControlResult =
                                runBackendAb(
                                    backendControlHPolys);
                        }

                        if (backendMetricCorridorSuccess)
                        {
                            backendMetricResult =
                                runBackendAb(
                                    backendMetricHPolys);
                        }

                        ROS_INFO_STREAM(
                            "TF_CSGN_BACKEND_AB "
                            << "budget="
                            << backendAbFaceBudget
                        
                            << " control_corridor_success="
                            << backendControlCorridorSuccess
                        
                            << " metric_corridor_success="
                            << backendMetricCorridorSuccess
                        
                            << " control_setup_success="
                            << backendControlResult.setup_success
                        
                            << " metric_setup_success="
                            << backendMetricResult.setup_success
                        
                            << " control_opt_success="
                            << backendControlResult.optimize_success
                        
                            << " metric_opt_success="
                            << backendMetricResult.optimize_success
                        
                            << " control_corridors="
                            << backendControlResult.corridor_count
                        
                            << " metric_corridors="
                            << backendMetricResult.corridor_count
                        
                            << " control_faces="
                            << backendControlResult.total_faces
                        
                            << " metric_faces="
                            << backendMetricResult.total_faces
                        
                            << " control_cost="
                            << backendControlResult.final_cost
                        
                            << " metric_cost="
                            << backendMetricResult.final_cost
                        
                            << " delta_cost="
                            << (backendMetricResult.final_cost -
                                backendControlResult.final_cost)
                            
                            << " control_setup_ms="
                            << backendControlResult.setup_ms
                            
                            << " metric_setup_ms="
                            << backendMetricResult.setup_ms
                            
                            << " control_opt_ms="
                            << backendControlResult.optimize_ms
                            
                            << " metric_opt_ms="
                            << backendMetricResult.optimize_ms
                            
                            << " control_duration="
                            << backendControlResult.trajectory_duration
                            
                            << " metric_duration="
                            << backendMetricResult.trajectory_duration
                            
                            << " control_penalty_initial="
                            << backendControlResult.corridor_penalty_initial
                            
                            << " metric_penalty_initial="
                            << backendMetricResult.corridor_penalty_initial
                            
                            << " control_penalty_final="
                            << backendControlResult.corridor_penalty_final
                            
                            << " metric_penalty_final="
                            << backendMetricResult.corridor_penalty_final
                            
                            << " control_violation_initial="
                            << backendControlResult.max_corridor_violation_initial
                            
                            << " metric_violation_initial="
                            << backendMetricResult.max_corridor_violation_initial
                            
                            << " control_violation_final="
                            << backendControlResult.max_corridor_violation_final
                            
                            << " metric_violation_final="
                            << backendMetricResult.max_corridor_violation_final

                            << " control_traj_pieces="
                            << backendControlResult.trajectory_pieces
                                                        
                            << " metric_traj_pieces="
                            << backendMetricResult.trajectory_pieces
                                                        
                            << " control_constrained_pieces="
                            << backendControlResult.constrained_pieces
                                                        
                            << " metric_constrained_pieces="
                            << backendMetricResult.constrained_pieces
                                                        
                            << " control_slack_initial="
                            << backendControlResult.corridor_slack_initial
                                                        
                            << " metric_slack_initial="
                            << backendMetricResult.corridor_slack_initial
                                                        
                            << " control_slack_final="
                            << backendControlResult.corridor_slack_final
                                                        
                            << " metric_slack_final="
                            << backendMetricResult.corridor_slack_final);

                        // ============================================================
                        // ============================================================
                        // MINCO-piece-native compact corridor.
                        //
                        // Unlike the route-segment MVP:
                        //
                        //   - no raw RRT segment is used,
                        //   - no MINCO -> route metric mapping is used,
                        //   - one corridor is attempted directly for each nominal
                        //     MINCO polynomial piece,
                        //   - complete continuous-time containment is certified by
                        //     exact directional support.
                        // ============================================================
                        std::vector<Eigen::MatrixX4d>
                            mincoNativeHPolys;

                        std::vector<
                            traj_relevant::
                                MincoPieceCorridorDiagnostics,
                            Eigen::aligned_allocator<
                                traj_relevant::
                                    MincoPieceCorridorDiagnostics>>
                            mincoNativeInfos;

                        bool mincoNativeSuccess =
                            success &&
                            traj.getPieceNum() ==
                                static_cast<int>(
                                    metrics.size());

                        int mincoNativeAdjacentOverlapCount =
                            0;

                        const auto mincoNativeStarted =
                            std::chrono::steady_clock::now();

                        if (mincoNativeSuccess)
                        {
                            mincoNativeHPolys.reserve(
                                traj.getPieceNum());

                            mincoNativeInfos.reserve(
                                traj.getPieceNum());

                            for (int pieceId = 0;
                                 pieceId < traj.getPieceNum();
                                 ++pieceId)
                            {
                                traj_relevant::
                                    MincoPieceCorridorOptions
                                        pieceOptions;

                                pieceOptions.max_extra_radius =
                                    std::max(
                                        config.tfFiriRange,
                                        config.voxelWidth);

                                pieceOptions.min_extra_ratio =
                                    0.25;

                                pieceOptions.overlap_radius =
                                    0.01;

                                pieceOptions.epsilon =
                                    1.0e-6;

                                pieceOptions.root_tolerance =
                                    1.0e-10;

                                pieceOptions.metric_enabled =
                                    metrics[pieceId].valid;

                                pieceOptions.deformation_utility =
                                    metrics[pieceId]
                                        .corridorUtility;

                                Eigen::MatrixX4d piecePoly;

                                traj_relevant::
                                    MincoPieceCorridorDiagnostics
                                        pieceInfo;

                                const bool pieceSuccess =
                                    traj_relevant::
                                        buildCompactMincoPiecePolytope(
                                            pc,
                                            voxelMap.getOrigin(),
                                            voxelMap.getCorner(),
                                            traj[pieceId],
                                            pieceOptions,
                                            piecePoly,
                                            &pieceInfo);

                                mincoNativeInfos.push_back(
                                    pieceInfo);

                                if (!pieceSuccess)
                                {
                                    mincoNativeSuccess =
                                        false;

                                    ROS_WARN_STREAM(
                                        "TF_MINCO_NATIVE_FAILURE "
                                        << "piece="
                                        << pieceId

                                        << " input_obs="
                                        << pieceInfo
                                               .input_obstacle_count

                                        << " local_obs="
                                        << pieceInfo
                                               .local_obstacle_count

                                        << " candidates="
                                        << pieceInfo
                                               .candidate_count

                                        << " rejected_candidates="
                                        << pieceInfo
                                               .rejected_candidate_count

                                        << " unresolved="
                                        << pieceInfo
                                               .unresolved_obstacle_count

                                        << " projection_valid="
                                        << pieceInfo
                                               .unresolved_projection_valid_count

                                        << " projection_separable="
                                        << pieceInfo
                                               .unresolved_certified_separable_count

                                        << " projection_ambiguous="
                                        << pieceInfo
                                               .unresolved_projection_ambiguous_count

                                        << " first_unresolved="
                                        << pieceInfo
                                               .first_unresolved_obstacle

                                        << " first_projection_valid="
                                        << pieceInfo
                                               .first_unresolved_projection_valid

                                        << " first_projection_converged="
                                        << pieceInfo
                                               .first_unresolved_projection_converged

                                        << " first_certified_separable="
                                        << pieceInfo
                                               .first_unresolved_certified_separable

                                        << " first_metric_d2="
                                        << pieceInfo
                                               .first_unresolved_metric_distance_squared

                                        << " first_metric_d2_lb="
                                        << pieceInfo
                                               .first_unresolved_metric_distance_lower_bound_squared

                                        << " first_euclidean_d="
                                        << pieceInfo
                                               .first_unresolved_euclidean_distance

                                        << " first_sep_margin_m="
                                        << pieceInfo
                                               .first_unresolved_separation_margin_m

                                        << " first_fw_gap="
                                        << pieceInfo
                                               .first_unresolved_fw_gap

                                        << " first_fw_iters="
                                        << pieceInfo
                                               .first_unresolved_fw_iterations);

                                    break;
                                }

                                if (!mincoNativeHPolys.empty())
                                {
                                    const bool adjacentOverlap =
                                        geo_utils::overlap(
                                            mincoNativeHPolys.back(),
                                            piecePoly,
                                            0.005);

                                    if (adjacentOverlap)
                                    {
                                        ++mincoNativeAdjacentOverlapCount;
                                    }
                                    else
                                    {
                                        mincoNativeSuccess =
                                            false;

                                        ROS_WARN_STREAM(
                                            "TF_MINCO_NATIVE_FAILURE "
                                            << "piece="
                                            << pieceId
                                            << " reason=adjacent_overlap");

                                        break;
                                    }
                                }

                                mincoNativeHPolys.push_back(
                                    piecePoly);
                            }
                        }

                        const double mincoNativeMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::steady_clock::now() -
                                    mincoNativeStarted)
                                .count();

                        int mincoNativeFaces =
                            0;

                        int mincoNativeDomainFaces =
                            0;

                        int mincoNativeObstacleFaces =
                            0;

                        int mincoNativeCandidates =
                            0;

                        int mincoNativeRejectedCandidates =
                            0;

                        int mincoNativeGreedyFaces =
                            0;

                        int mincoNativeRedundancyRemoved =
                            0;

                        int mincoNativeTrajectoryContained =
                            0;

                        int mincoNativeSafetyVerified =
                            0;

                        double mincoNativeWeightedDamage =
                            0.0;

                        for (const auto &info :
                             mincoNativeInfos)
                        {
                            mincoNativeFaces +=
                                info.total_face_count;

                            mincoNativeDomainFaces +=
                                info.domain_face_count;

                            mincoNativeObstacleFaces +=
                                info.selected_obstacle_face_count;

                            mincoNativeCandidates +=
                                info.candidate_count;

                            mincoNativeRejectedCandidates +=
                                info.rejected_candidate_count;

                            mincoNativeGreedyFaces +=
                                info.greedy_obstacle_face_count;

                            mincoNativeRedundancyRemoved +=
                                info.redundancy_removed;

                            mincoNativeTrajectoryContained +=
                                info.trajectory_contained
                                    ? 1
                                    : 0;

                            mincoNativeSafetyVerified +=
                                info.safety_verified
                                    ? 1
                                    : 0;

                            mincoNativeWeightedDamage +=
                                static_cast<double>(
                                    info.selected_obstacle_face_count) *
                                info.mean_metric_damage;
                        }

                        const double mincoNativeMeanDamage =
                            mincoNativeObstacleFaces > 0
                                ? mincoNativeWeightedDamage /
                                      static_cast<double>(
                                          mincoNativeObstacleFaces)
                                : 0.0;

                        ROS_INFO_STREAM(
                            "TF_MINCO_NATIVE_AB "
                            << "success="
                            << mincoNativeSuccess

                            << " nominal_pieces="
                            << traj.getPieceNum()

                            << " completed_corridors="
                            << mincoNativeHPolys.size()

                            << " info_count="
                            << mincoNativeInfos.size()

                            << " control_firi_faces="
                            << controlFaceCount

                            << " route_compact_faces="
                            << compactTotalFaces

                            << " minco_faces="
                            << mincoNativeFaces

                            << " minco_domain_faces="
                            << mincoNativeDomainFaces

                            << " minco_obs_faces="
                            << mincoNativeObstacleFaces

                            << " candidates="
                            << mincoNativeCandidates

                            << " rejected_candidates="
                            << mincoNativeRejectedCandidates

                            << " greedy_faces="
                            << mincoNativeGreedyFaces

                            << " redundancy_removed="
                            << mincoNativeRedundancyRemoved

                            << " mean_psi="
                            << mincoNativeMeanDamage

                            << " trajectory_contained="
                            << mincoNativeTrajectoryContained
                            << "/"
                            << mincoNativeInfos.size()

                            << " safety="
                            << mincoNativeSafetyVerified
                            << "/"
                            << mincoNativeInfos.size()

                            << " adjacent_overlap="
                            << mincoNativeAdjacentOverlapCount
                            << "/"
                            << std::max(
                                   0,
                                   static_cast<int>(
                                       mincoNativeHPolys.size()) -
                                       1)

                            << " generation_ms="
                            << mincoNativeMs);

                        // ============================================================
                        // MINCO-piece-native corridor -> GCOPTER backend.
                        //
                        // The SAME runBackendAb() routine is used for:
                        //
                        //   - hard-budget FIRI CONTROL,
                        //   - hard-budget FIRI METRIC,
                        //   - route-segment compact corridor,
                        //   - MINCO-piece-native compact corridor.
                        //
                        // Therefore setup/optimization diagnostics are directly
                        // comparable at the backend implementation level.
                        // ============================================================
                        BackendAbResult
                            mincoNativeBackendResult;

                        if (mincoNativeSuccess &&
                            mincoNativeHPolys.size() ==
                                static_cast<std::size_t>(
                                    traj.getPieceNum()))
                        {
                            mincoNativeBackendResult =
                                runBackendAb(
                                    mincoNativeHPolys);
                        }

                        ROS_INFO_STREAM(
                            "TF_MINCO_NATIVE_BACKEND "
                            << "corridor_success="
                            << mincoNativeSuccess

                            << " setup_success="
                            << mincoNativeBackendResult.setup_success

                            << " opt_success="
                            << mincoNativeBackendResult.optimize_success

                            << " corridors="
                            << mincoNativeBackendResult.corridor_count

                            << " faces="
                            << mincoNativeBackendResult.total_faces

                            << " traj_pieces="
                            << mincoNativeBackendResult.trajectory_pieces

                            << " constrained_pieces="
                            << mincoNativeBackendResult.constrained_pieces

                            << " final_cost="
                            << mincoNativeBackendResult.final_cost

                            << " duration="
                            << mincoNativeBackendResult.trajectory_duration

                            << " setup_ms="
                            << mincoNativeBackendResult.setup_ms

                            << " opt_ms="
                            << mincoNativeBackendResult.optimize_ms

                            << " penalty_initial="
                            << mincoNativeBackendResult.corridor_penalty_initial

                            << " penalty_final="
                            << mincoNativeBackendResult.corridor_penalty_final

                            << " violation_initial="
                            << mincoNativeBackendResult.max_corridor_violation_initial

                            << " violation_final="
                            << mincoNativeBackendResult.max_corridor_violation_final

                            << " slack_initial="
                            << mincoNativeBackendResult.corridor_slack_initial

                            << " slack_final="
                            << mincoNativeBackendResult.corridor_slack_final

                            // Same-run B=28 CONTROL reference.
                            << " control_faces="
                            << backendControlResult.total_faces

                            << " control_cost="
                            << backendControlResult.final_cost

                            << " control_setup_ms="
                            << backendControlResult.setup_ms

                            << " control_opt_ms="
                            << backendControlResult.optimize_ms

                            << " control_duration="
                            << backendControlResult.trajectory_duration);

                        // Trajectory-relevant compact corridor -> GCOPTER backend.
                        //
                        // Reuse exactly the same backend diagnostic routine as the
                        // existing paired CSGN experiment.
                        //
                        // This directly tests whether the large face-count reduction
                        // translates into lower optimization burden without degrading
                        // trajectory quality or corridor feasibility.
                        // ============================================================
                        BackendAbResult
                            compactBackendResult;

                        if (compactSuccess)
                        {
                            compactBackendResult =
                                runBackendAb(
                                    compactHPolys);
                        }

                        ROS_INFO_STREAM(
                            "TF_MINFACE_BACKEND "
                            << "corridor_success="
                            << compactSuccess

                            << " setup_success="
                            << compactBackendResult.setup_success

                            << " opt_success="
                            << compactBackendResult.optimize_success

                            << " corridors="
                            << compactBackendResult.corridor_count

                            << " faces="
                            << compactBackendResult.total_faces

                            << " traj_pieces="
                            << compactBackendResult.trajectory_pieces

                            << " constrained_pieces="
                            << compactBackendResult.constrained_pieces

                            << " final_cost="
                            << compactBackendResult.final_cost

                            << " duration="
                            << compactBackendResult.trajectory_duration

                            << " setup_ms="
                            << compactBackendResult.setup_ms

                            << " opt_ms="
                            << compactBackendResult.optimize_ms

                            << " penalty_initial="
                            << compactBackendResult.corridor_penalty_initial

                            << " penalty_final="
                            << compactBackendResult.corridor_penalty_final

                            << " violation_initial="
                            << compactBackendResult.max_corridor_violation_initial

                            << " violation_final="
                            << compactBackendResult.max_corridor_violation_final

                            << " slack_initial="
                            << compactBackendResult.corridor_slack_initial

                            << " slack_final="
                            << compactBackendResult.corridor_slack_final);

                        const int minimumFaceBudgetToTest =
                            24;

                        const int maximumFaceBudgetToTest =
                            40;

                        // We call this "first successful budget" rather than a
                        // theoretical minimum because the heuristic candidate
                        // selection itself changes with the available budget.
                        int controlFirstSuccessfulBudget =
                            -1;

                        int metricFirstSuccessfulBudget =
                            -1;

                        for (int faceBudget =
                                 minimumFaceBudgetToTest;
                             faceBudget <=
                                 maximumFaceBudgetToTest;
                             ++faceBudget)
                        {
                            const FaceBudgetRunSummary
                                controlBudgetRun =
                                    runFaceBudgetCase(
                                        faceBudget,
                                        0.0);

                            const FaceBudgetRunSummary
                                metricBudgetRun =
                                    runFaceBudgetCase(
                                        faceBudget,
                                        1.0);

                            if (controlBudgetRun.success &&
                                controlFirstSuccessfulBudget < 0)
                            {
                                controlFirstSuccessfulBudget =
                                    faceBudget;
                            }

                            if (metricBudgetRun.success &&
                                metricFirstSuccessfulBudget < 0)
                            {
                                metricFirstSuccessfulBudget =
                                    faceBudget;
                            }

                            double deltaMeanMetricDamage =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            if (controlBudgetRun.success &&
                                metricBudgetRun.success &&
                                std::isfinite(
                                    controlBudgetRun.meanMetricDamage) &&
                                std::isfinite(
                                    metricBudgetRun.meanMetricDamage))
                            {
                                deltaMeanMetricDamage =
                                    metricBudgetRun.meanMetricDamage -
                                    controlBudgetRun.meanMetricDamage;
                            }

                            ROS_INFO_STREAM(
                                "TF_CSGN_FIRI_BUDGET "
                                << "budget="
                                << faceBudget

                                << " control_success="
                                << controlBudgetRun.success

                                << " metric_success="
                                << metricBudgetRun.success

                                << " control_corridors="
                                << controlBudgetRun.corridorCount

                                << " metric_corridors="
                                << metricBudgetRun.corridorCount

                                << " control_faces="
                                << controlBudgetRun.totalFaces

                                << " metric_faces="
                                << metricBudgetRun.totalFaces

                                << " control_obs_faces="
                                << controlBudgetRun.obstacleFaces

                                << " metric_obs_faces="
                                << metricBudgetRun.obstacleFaces

                                << " control_mean_psi="
                                << controlBudgetRun.meanMetricDamage

                                << " metric_mean_psi="
                                << metricBudgetRun.meanMetricDamage

                                << " delta_mean_psi="
                                << deltaMeanMetricDamage

                                << " control_ms="
                                << controlBudgetRun.elapsedMs

                                << " metric_ms="
                                << metricBudgetRun.elapsedMs

                                << " control_fail_face="
                                << controlBudgetRun.failureFaceCount

                                << " metric_fail_face="
                                << metricBudgetRun.failureFaceCount

                                << " control_unresolved_obs="
                                << controlBudgetRun.unresolvedObstacle

                                << " metric_unresolved_obs="
                                << metricBudgetRun.unresolvedObstacle

                                << " control_budget_saturated="
                                << controlBudgetRun.failureBudgetSaturated

                                << " metric_budget_saturated="
                                << metricBudgetRun.failureBudgetSaturated

                                << " control_exchange_attempted="
                                << controlBudgetRun.exchangeAttempted

                                << " metric_exchange_attempted="
                                << metricBudgetRun.exchangeAttempted

                                << " control_exchange_accepted="
                                << controlBudgetRun.exchangeAccepted

                                << " metric_exchange_accepted="
                                << metricBudgetRun.exchangeAccepted);
                        }

                        ROS_INFO_STREAM(
                            "TF_CSGN_FIRI_BUDGET_THRESHOLD "
                            << "control_first_success="
                            << controlFirstSuccessfulBudget

                            << " metric_first_success="
                            << metricFirstSuccessfulBudget

                            << " tested_min="
                            << minimumFaceBudgetToTest

                            << " tested_max="
                            << maximumFaceBudgetToTest);

                        // ============================================================
                        // Metric-weight sweep around the empirical feasibility
                        // threshold.
                        //
                        // The previous face-budget sweep showed:
                        //
                        //   CONTROL first full success: B = 28
                        //   METRIC  first full success: B = 29  (for metric_weight=1)
                        //
                        // We now test whether the CSGN term is simply too strong
                        // relative to the coverage/face-count term.
                        //
                        // Nothing except metric_weight changes inside each fixed
                        // face-budget experiment.
                        // ============================================================
                        const std::vector<int>
                            metricWeightProbeBudgets =
                                {
                                    28,
                                    29
                                };
                            
                        const std::vector<double>
                            metricWeightsToTest =
                                {
                                    0.0,
                                    0.1,
                                    0.25,
                                    0.5,
                                    0.75,
                                    1.0
                                };
                            
                        for (const int faceBudget :
                             metricWeightProbeBudgets)
                        {
                            // metric_weight = 0 is the exact CONTROL reference
                            // under this same per-region face budget.
                            const FaceBudgetRunSummary
                                weightSweepControl =
                                    runFaceBudgetCase(
                                        faceBudget,
                                        0.0);
                                    
                            for (const double metricWeight :
                                 metricWeightsToTest)
                            {
                                const FaceBudgetRunSummary
                                    weightSweepRun =
                                        runFaceBudgetCase(
                                            faceBudget,
                                            metricWeight);
                                        
                                int deltaTotalFaces =
                                    0;
                                        
                                int deltaObstacleFaces =
                                    0;
                                        
                                double deltaMeanMetricDamage =
                                    std::numeric_limits<double>::
                                        quiet_NaN();
                                        
                                if (weightSweepControl.success &&
                                    weightSweepRun.success)
                                {
                                    deltaTotalFaces =
                                        weightSweepRun.totalFaces -
                                        weightSweepControl.totalFaces;
                                
                                    deltaObstacleFaces =
                                        weightSweepRun.obstacleFaces -
                                        weightSweepControl.obstacleFaces;
                                
                                    if (std::isfinite(
                                            weightSweepControl.meanMetricDamage) &&
                                        std::isfinite(
                                            weightSweepRun.meanMetricDamage))
                                    {
                                        deltaMeanMetricDamage =
                                            weightSweepRun.meanMetricDamage -
                                            weightSweepControl.meanMetricDamage;
                                    }
                                }
                            
                                ROS_INFO_STREAM(
                                    "TF_CSGN_FIRI_WEIGHT "
                                    << "budget="
                                    << faceBudget
                                
                                    << " weight="
                                    << metricWeight
                                
                                    << " control_success="
                                    << weightSweepControl.success
                                
                                    << " success="
                                    << weightSweepRun.success
                                
                                    << " corridors="
                                    << weightSweepRun.corridorCount
                                
                                    << " infos="
                                    << weightSweepRun.infoCount
                                
                                    << " faces="
                                    << weightSweepRun.totalFaces
                                
                                    << " obs_faces="
                                    << weightSweepRun.obstacleFaces
                                
                                    << " mean_psi="
                                    << weightSweepRun.meanMetricDamage
                                
                                    << " delta_faces="
                                    << deltaTotalFaces
                                
                                    << " delta_obs_faces="
                                    << deltaObstacleFaces
                                
                                    << " delta_mean_psi="
                                    << deltaMeanMetricDamage
                                
                                    << " unresolved_total="
                                    << weightSweepRun.unresolvedTotal
                                
                                    << " unresolved_obs="
                                    << weightSweepRun.unresolvedObstacle
                                
                                    << " budget_saturated="
                                    << weightSweepRun.failureBudgetSaturated
                                
                                    << " exchange_attempted="
                                    << weightSweepRun.exchangeAttempted
                                
                                    << " exchange_accepted="
                                    << weightSweepRun.exchangeAccepted
                                
                                    << " elapsed_ms="
                                    << weightSweepRun.elapsedMs);
                            }
                        }
                    }

                    for (size_t pieceId = 0;
                         pieceId < metrics.size();
                         ++pieceId)
                    {
                        const auto &metric =
                            metrics[pieceId];
                    
                        if (!metric.valid)
                        {
                            ROS_WARN_STREAM(
                                "TF_CSGN_COMPRESS "
                                << "piece=" << pieceId
                                << " INVALID reason="
                                << metric.failureReason);
                            
                            continue;
                        }
                    
                        ROS_INFO_STREAM(
                            "TF_CSGN_COMPRESS "
                            << "piece=" << pieceId
                        
                            << " natural_anis="
                            << metric.anisotropy
                        
                            << " alpha="
                            << metric.spectrumCompressionAlpha
                        
                            << " corridor_anis="
                            << metric.corridorAnisotropy
                        
                            << " natural_eig=["
                            << metric.utilityEigenvalues(0)
                            << ","
                            << metric.utilityEigenvalues(1)
                            << ","
                            << metric.utilityEigenvalues(2)
                            << "]"
                        
                            << " corridor_eig=["
                            << metric.corridorUtilityEigenvalues(0)
                            << ","
                            << metric.corridorUtilityEigenvalues(1)
                            << ","
                            << metric.corridorUtilityEigenvalues(2)
                            << "]"
                        
                            << " det_natural="
                            << metric.utility.determinant()
                        
                            << " det_corridor="
                            << metric.corridorUtility.determinant());
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
