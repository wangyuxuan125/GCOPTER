#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/minco.hpp"
#include "gcopter/route_minco_guide.hpp"
#include "gcopter/minco_affine_map.hpp"
#include "gcopter/exact_sfc_projection.hpp"
#include "gcopter/bernstein_sfc_projection.hpp"
#include "gcopter/lazy_bernstein_projection.hpp"
#include "gcopter/minco_support.hpp"
#include "gcopter/minco_piece_corridor.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/experiment_logger.hpp"
#include "gcopter/route_replay.hpp"
#include "gcopter/benchmark_logger.hpp"
#include "gcopter/trajectory_metrics.hpp"
#include "gcopter/corridor_metrics.hpp"
#include "gcopter/effective_face_metrics.hpp"
#include "gcopter/corridor_safety_metrics.hpp"
#include "gcopter/rils_baseline.hpp"
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
    std::string benchmarkCaseId;
    std::string benchmarkEnvironmentFamily;
    std::string benchmarkDifficulty;
    int benchmarkRepeatId;
    bool benchmarkEnabled;
    std::string benchmarkMethod;
    std::string benchmarkVariant;
    bool benchmarkRouteReplayEnabled;
    std::string benchmarkRouteReplayFile;
    bool benchmarkRouteSaveEnabled;
    std::string benchmarkRouteSaveFile;
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
        nh_priv.param<std::string>(
            "Benchmark/CaseId",
            benchmarkCaseId,
            "");
        nh_priv.param<std::string>(
            "Benchmark/EnvironmentFamily",
            benchmarkEnvironmentFamily,
            "mockamap");
        nh_priv.param<std::string>(
            "Benchmark/Difficulty",
            benchmarkDifficulty,
            "development");
        nh_priv.param(
            "Benchmark/RepeatId",
            benchmarkRepeatId,
            0);
        nh_priv.param(
            "Benchmark/Enabled",
            benchmarkEnabled,
            false);

        nh_priv.param<std::string>(
            "Benchmark/Method",
            benchmarkMethod,
            "proposed");
        
        nh_priv.param<std::string>(
            "Benchmark/Variant",
            benchmarkVariant,
            "csgn_active_exact");
        nh_priv.param(
            "Benchmark/RouteReplayEnabled",
            benchmarkRouteReplayEnabled,
            false);
        nh_priv.param<std::string>(
            "Benchmark/RouteReplayFile",
            benchmarkRouteReplayFile,
            "");
        nh_priv.param(
            "Benchmark/RouteSaveEnabled",
            benchmarkRouteSaveEnabled,
            false);
        nh_priv.param<std::string>(
            "Benchmark/RouteSaveFile",
            benchmarkRouteSaveFile,
            "");
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
    gcopter_benchmark::CaseCsvLogger benchmarkCaseLogger;
    gcopter_benchmark::RunCsvLogger benchmarkRunLogger;
    gcopter_benchmark::CorridorCsvLogger benchmarkCorridorLogger;
public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          fixedPlanTriggered(false),
          visualizer(nh),
          experimentLogger(
              config.experimentLogEnabled,
              config.experimentLogDirectory,
              config.experimentTag),
          
          benchmarkCaseLogger(
              config.experimentLogEnabled,
              config.experimentLogDirectory),
          
          benchmarkRunLogger(
              config.experimentLogEnabled,
              config.experimentLogDirectory),
                  
          benchmarkCorridorLogger(
              config.experimentLogEnabled,
              config.experimentLogDirectory)
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

            // ============================================================
            // Deterministic benchmark route infrastructure.
            //
            // Generation mode:
            //     RRT -> validate -> optionally save.
            //
            // Replay mode:
            //     load fixed route -> validate -> downstream benchmark.
            //
            // The route itself is excluded from the paired comparison
            // between different SFC methods.
            // ============================================================
            std::vector<Eigen::Vector3d>
                route;
                    
            bool routeLoadedFromReplay =
                false;
                    
            bool routeSaved =
                false;
                    
            double routeIoMs =
                0.0;
                    
            if (config.benchmarkRouteReplayEnabled &&
                config.benchmarkRouteSaveEnabled)
            {
                ROS_ERROR(
                    "Benchmark route replay and route save "
                    "cannot be enabled simultaneously.");
                
                finishRecord(
                    "benchmark_route_mode_conflict",
                    false);
                
                return;
            }
            
            if (config.benchmarkRouteReplayEnabled)
            {
                if (config
                        .benchmarkRouteReplayFile
                        .empty())
                {
                    ROS_ERROR(
                        "Benchmark route replay enabled "
                        "but replay file is empty.");
                    
                    finishRecord(
                        "route_replay_file_missing",
                        false);
                    
                    return;
                }
            
                const auto ioStarted =
                    std::chrono::
                        steady_clock::now();
            
                routeLoadedFromReplay =
                    gcopter_benchmark::
                        loadRoute(
                            config
                                .benchmarkRouteReplayFile,
                            route);
                        
                routeIoMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::
                                steady_clock::now() -
                            ioStarted)
                        .count();
                        
                // Route loading is benchmark input preparation,
                // NOT path-search runtime.
                record.path_search_ms =
                    0.0;
                        
                if (!routeLoadedFromReplay)
                {
                    ROS_ERROR_STREAM(
                        "Failed to load benchmark route: "
                        << config
                               .benchmarkRouteReplayFile);
                    
                    finishRecord(
                        "route_replay_load_failure",
                        false);
                    
                    return;
                }
            }
            else
            {
                const auto pathStarted =
                    std::chrono::
                        steady_clock::now();
            
                sfc_gen::
                    planPath<
                        voxel_map::VoxelMap>(
                            startGoal[0],
                            startGoal[1],
                            voxelMap
                                .getOrigin(),
                            voxelMap
                                .getCorner(),
                            &voxelMap,
                            config.timeoutRRT,
                            route,
                            static_cast<
                                std::uint_fast32_t>(
                                    std::max(
                                        config
                                            .routeSeed,
                                        0)));
                                    
                record.path_search_ms =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::
                                steady_clock::now() -
                            pathStarted)
                        .count();
            }
            
            record.route_point_count =
                static_cast<int>(
                    route.size());
                
            if (route.size() <= 1)
            {
                finishRecord(
                    routeLoadedFromReplay
                        ? "route_replay_invalid_size"
                        : "path_search_failure",
                    false);
                
                return;
            }
            
            // ------------------------------------------------------------
            // The replayed route must correspond to the same start/goal
            // state used by the benchmark case.
            // ------------------------------------------------------------
            constexpr double
                endpointToleranceM =
                    1.0e-6;
            
            const double startMismatch =
                (route.front() -
                 startGoal[0])
                    .norm();
            
            const double goalMismatch =
                (route.back() -
                 startGoal[1])
                    .norm();
            
            if (startMismatch >
                    endpointToleranceM ||
                goalMismatch >
                    endpointToleranceM)
            {
                ROS_ERROR_STREAM(
                    "Benchmark route endpoint mismatch: "
                    << "start_error_m="
                    << startMismatch
                    << " goal_error_m="
                    << goalMismatch);
                
                finishRecord(
                    "route_endpoint_mismatch",
                    false);
                
                return;
            }
            
            // ------------------------------------------------------------
            // Validate the complete piecewise-linear route against the
            // CURRENT dilated voxel map.
            //
            // This is benchmark-input validation and is NOT included in
            // path_search_ms or after-route planning time.
            // ------------------------------------------------------------
            const double routeValidationStepM =
                std::max(
                    0.25 *
                        config.voxelWidth,
                    1.0e-4);
                
            const auto routeValidation =
                gcopter_benchmark::
                    validateRoute(
                        route,
                        voxelMap,
                        routeValidationStepM);
                    
            if (!routeValidation.valid)
            {
                ROS_ERROR_STREAM(
                    "Benchmark route validation failed: "
                    << "occupied_samples="
                    << routeValidation
                           .occupied_samples
                    << " first_bad_segment="
                    << routeValidation
                           .first_bad_segment);
                
                finishRecord(
                    routeLoadedFromReplay
                        ? "route_replay_collision"
                        : "generated_route_validation_failure",
                    false);
                
                return;
            }
            
            const std::string
                routeFingerprint =
                    gcopter_benchmark::
                        routeFingerprint(
                            route);
                        
            // ------------------------------------------------------------
            // Create a deterministic case ID when the caller does not
            // explicitly provide one.
            // ------------------------------------------------------------
            const std::string
                effectiveCaseId =
                    config
                            .benchmarkCaseId
                            .empty()
                        ? (
                              config
                                  .benchmarkEnvironmentFamily +
                              "_" +
                              config
                                  .benchmarkDifficulty +
                              "_m" +
                              std::to_string(
                                  config.mapSeed) +
                              "_r" +
                              std::to_string(
                                  config.routeSeed))
                        : config
                              .benchmarkCaseId;
                            
            // ------------------------------------------------------------
            // Save the generated route when creating the route bank.
            // ------------------------------------------------------------
            if (config.benchmarkRouteSaveEnabled)
            {
                if (config
                        .benchmarkRouteSaveFile
                        .empty())
                {
                    ROS_ERROR(
                        "Benchmark route save enabled "
                        "but save file is empty.");
                    
                    finishRecord(
                        "route_save_file_missing",
                        false);
                    
                    return;
                }
            
                const auto ioStarted =
                    std::chrono::
                        steady_clock::now();
            
                routeSaved =
                    gcopter_benchmark::
                        saveRoute(
                            config
                                .benchmarkRouteSaveFile,
                            route);
                        
                routeIoMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::
                                steady_clock::now() -
                            ioStarted)
                        .count();
                        
                if (!routeSaved)
                {
                    ROS_ERROR_STREAM(
                        "Failed to save benchmark route: "
                        << config
                               .benchmarkRouteSaveFile);
                    
                    finishRecord(
                        "route_save_failure",
                        false);
                    
                    return;
                }
            
                // --------------------------------------------------------
                // One case-table row is created when the immutable route
                // bank entry is created.
                // --------------------------------------------------------
                gcopter_benchmark::
                    CaseRecord
                        caseRecord;
            
                caseRecord.case_id =
                    effectiveCaseId;
            
                caseRecord
                    .environment_family =
                        config
                            .benchmarkEnvironmentFamily;
            
                caseRecord.difficulty =
                    config
                        .benchmarkDifficulty;
            
                caseRecord.map_id =
                    "mockamap_seed_" +
                    std::to_string(
                        config.mapSeed);
                    
                caseRecord.map_seed =
                    config.mapSeed;
                    
                caseRecord.route_id =
                    effectiveCaseId;
                    
                caseRecord.source_route_seed =
                    config.routeSeed;
                    
                caseRecord.route_file =
                    config
                        .benchmarkRouteSaveFile;
                    
                caseRecord.route_fingerprint =
                    routeFingerprint;
                    
                caseRecord.route_length_m =
                    routeValidation
                        .route_length_m;
                    
                caseRecord.route_point_count =
                    routeValidation
                        .point_count;
                    
                caseRecord.route_segment_count =
                    routeValidation
                        .segment_count;
                    
                caseRecord.start_x =
                    startGoal[0].x();
                    
                caseRecord.start_y =
                    startGoal[0].y();
                    
                caseRecord.start_z =
                    startGoal[0].z();
                    
                caseRecord.goal_x =
                    startGoal[1].x();
                    
                caseRecord.goal_y =
                    startGoal[1].y();
                    
                caseRecord.goal_z =
                    startGoal[1].z();
                    
                caseRecord.voxel_width_m =
                    config.voxelWidth;
                    
                caseRecord.dilate_radius_m =
                    config.dilateRadius;
                    
                caseRecord
                    .route_validation_success =
                        routeValidation.valid;
                    
                caseRecord
                    .route_validation_samples =
                        routeValidation
                            .checked_samples;
                    
                caseRecord
                    .max_route_segment_m =
                        routeValidation
                            .max_segment_length_m;
                    
                caseRecord
                    .creation_timestamp_s =
                        ros::Time::
                            now()
                                .toSec();
                    
                if (!benchmarkCaseLogger
                         .logCase(
                             caseRecord))
                {
                    ROS_ERROR(
                        "Failed to append "
                        "benchmark_cases_v1.csv.");
                    
                    finishRecord(
                        "benchmark_case_log_failure",
                        false);
                    
                    return;
                }
            }
            
            // ------------------------------------------------------------
            // Single diagnostic used to verify route-bank generation and
            // deterministic replay.
            // ------------------------------------------------------------
            ROS_INFO_STREAM(
                "TF_ROUTE_SOURCE "
                << "case_id="
                << effectiveCaseId
            
                << " mode="
                << (routeLoadedFromReplay
                        ? "replay"
                        : "generated")
                
                << " route_file="
                << (routeLoadedFromReplay
                        ? config
                              .benchmarkRouteReplayFile
                        : config
                              .benchmarkRouteSaveFile)
                
                << " fingerprint="
                << routeFingerprint
                
                << " points="
                << route.size()
                
                << " segments="
                << routeValidation
                       .segment_count
                
                << " length_m="
                << routeValidation
                       .route_length_m
                
                << " max_segment_m="
                << routeValidation
                       .max_segment_length_m
                
                << " validation_success="
                << routeValidation.valid
                
                << " validation_samples="
                << routeValidation
                       .checked_samples
                
                << " validation_ms="
                << routeValidation
                       .validation_ms
                
                << " route_io_ms="
                << routeIoMs
                
                << " path_search_ms="
                << record
                       .path_search_ms);

            // ============================================================
            // RRT -> direct MINCO guide experiment.
            //
            // IMPORTANT:
            // This is currently diagnostic only.  The existing baseline
            // FIRI + GCOPTER pipeline below is NOT changed.
            //
            // Goal:
            //
            //   collision-free RRT polyline
            //           ->
            //   direct quintic MINCO interpolation
            //
            // without constructing a safe flight corridor first.
            //
            // If this guide is cheap and geometrically reasonable, it can
            // later replace the expensive FIRI-derived nominal trajectory
            // used only for trajectory sensitivity estimation.
            // ============================================================
            
            // ============================================================
            // Benchmark execution gates.
            //
            // benchmarkProposedMode:
            //     paper benchmark execution of the proposed pipeline.
            //
            // legacyDebugMode:
            //     historical diagnostic/A-B experiments.
            //
            // runProposedCore:
            //     functionality required by the proposed algorithm itself.
            // ============================================================
            const bool benchmarkProposedMode =
                config.benchmarkEnabled &&
                config.benchmarkMethod ==
                    "proposed";
                            
            const bool legacyDebugMode =
                !config.benchmarkEnabled &&
                config.experimentTag ==
                    "debug_metric";
                            
            const bool runProposedCore =
                benchmarkProposedMode ||
                legacyDebugMode;

            const auto proposedPipelineWallStarted =
                std::chrono::steady_clock::now();
            
            Trajectory<5> routeMincoGuide;

            bool routeMincoGuideValid =
                false;

            double routeMincoGuideBuildMs =
                0.0;
            
            Eigen::Matrix3Xd
                routeMincoGuideInnerPoints;
                    
            Eigen::VectorXd
                routeMincoGuideTimes;
                    
            Eigen::Matrix3d
                routeMincoGuideHeadPVA =
                    Eigen::Matrix3d::Zero();
                    
            Eigen::Matrix3d
                routeMincoGuideTailPVA =
                    Eigen::Matrix3d::Zero();

            if (runProposedCore)
            {
                // --------------------------------------------------------
                // Full direct-MINCO probe construction timer.
                //
                // Includes:
                //   waypoint preparation
                //   piece-time allocation
                //   boundary-state preparation
                //   MINCO construction
                //   basic trajectory validity verification
                //
                // Excludes all diagnostic sampling below.
                // --------------------------------------------------------
                const auto guideBuildStarted =
                    std::chrono::steady_clock::now();

                const int guidePieceCount =
                    static_cast<int>(
                        route.size()) -
                    1;

                // --------------------------------------------------------
                // Initial MINCO time allocation.
                //
                // This is NOT optimized.  Time is proportional to RRT
                // segment length using a conservative reference speed.
                //
                // Uniform time scaling primarily controls the dynamic
                // scale; this experiment is intended to evaluate the raw
                // trajectory geometry first.
                // --------------------------------------------------------
                const double guideReferenceSpeed =
                    std::max(
                        0.5 *
                            config.maxVelMag,
                        1.0e-3);

                routeMincoGuideInnerPoints.resize(
                    3,
                    std::max(
                        guidePieceCount - 1,
                        0));

                for (int pointId = 1;
                     pointId <
                         static_cast<int>(
                             route.size()) -
                             1;
                     ++pointId)
                {
                    routeMincoGuideInnerPoints.col(
                        pointId - 1) =
                        route[
                            pointId];
                }

                routeMincoGuideTimes.resize(
                    guidePieceCount);

                double routeLength =
                    0.0;

                double minGuideTime =
                    std::numeric_limits<double>::
                        infinity();

                double maxGuideTime =
                    0.0;

                for (int pieceId = 0;
                     pieceId <
                         guidePieceCount;
                     ++pieceId)
                {
                    const double segmentLength =
                        (route[
                             pieceId + 1] -
                         route[
                             pieceId])
                            .norm();

                    routeLength +=
                        segmentLength;

                    // 1e-3 is only a numerical floor, not a tuned
                    // trajectory-planning parameter.
                    routeMincoGuideTimes(
                        pieceId) =
                        std::max(
                            segmentLength /
                                guideReferenceSpeed,
                            1.0e-3);

                    minGuideTime =
                        std::min(
                            minGuideTime,
                            routeMincoGuideTimes(
                                pieceId));

                    maxGuideTime =
                        std::max(
                            maxGuideTime,
                            routeMincoGuideTimes(
                                pieceId));
                }

                routeMincoGuideHeadPVA =
                    Eigen::Matrix3d::Zero();

                routeMincoGuideTailPVA =
                    Eigen::Matrix3d::Zero();

                routeMincoGuideHeadPVA.col(0) =
                    route.front();

                routeMincoGuideTailPVA.col(0) =
                    route.back();

                minco::MINCO_S3NU guideMinco;

                guideMinco.setConditions(
                    routeMincoGuideHeadPVA,
                    routeMincoGuideTailPVA,
                    guidePieceCount);

                guideMinco.setParameters(
                    routeMincoGuideInnerPoints,
                    routeMincoGuideTimes);

                guideMinco.getTrajectory(
                    routeMincoGuide);

                double guideEnergy =
                    std::numeric_limits<double>::
                        infinity();

                guideMinco.getEnergy(
                    guideEnergy);

                routeMincoGuideValid =
                    routeMincoGuide.getPieceNum() ==
                        guidePieceCount &&
                    guidePieceCount > 0;

                if (routeMincoGuideValid)
                {
                    for (int pieceId = 0;
                         pieceId <
                             routeMincoGuide
                                 .getPieceNum();
                         ++pieceId)
                    {
                        const auto &piece =
                            routeMincoGuide[
                                pieceId];

                        if (!std::isfinite(
                                piece.getDuration()) ||
                            piece.getDuration() <=
                                0.0 ||
                            !piece.getCoeffMat()
                                 .allFinite())
                        {
                            routeMincoGuideValid =
                                false;

                            break;
                        }
                    }
                }

                // End the core guide timer BEFORE any diagnostic sampling.
                routeMincoGuideBuildMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::steady_clock::now() -
                            guideBuildStarted)
                        .count();

                if (legacyDebugMode)
                {
                    // ========================================================
                    // Guide diagnostics.
                    //
                    // These diagnostics are NOT counted in guideBuildMs.
                    //
                    // Exact:
                    //   maximum velocity norm
                    //   maximum acceleration norm
                    //
                    // Sampled:
                    //   voxel collision
                    //   body rate / tilt / thrust
                    //
                    // Sample spacing is chosen from the exact piece maximum
                    // velocity so that fast polynomial excursions receive more
                    // samples.  This collision test is diagnostic, NOT a
                    // continuous-time collision certificate.
                    // ========================================================
                    const auto guideDiagnosticStarted =
                        std::chrono::steady_clock::now();

                    int guideTotalSamples =
                        0;

                    int guideCollisionSamples =
                        0;

                    int guideCollisionPieces =
                        0;

                    int guideSamplingCapHits =
                        0;

                    double guideMaxVel =
                        0.0;

                    double guideMaxAcc =
                        0.0;

                    double guideMaxBodyRate =
                        0.0;

                    double guideMaxTilt =
                        0.0;

                    double guideMinThrust =
                        std::numeric_limits<double>::
                            infinity();

                    double guideMaxThrust =
                        -std::numeric_limits<double>::
                            infinity();

                    bool guideFlatnessValid =
                        true;

                    flatness::FlatnessMap
                        guideFlatness;

                    guideFlatness.reset(
                        config.vehicleMass,
                        config.gravAcc,
                        config.horizDrag,
                        config.vertDrag,
                        config.parasDrag,
                        config.speedEps);

                    if (routeMincoGuideValid)
                    {
                        const double spatialSampleStep =
                            std::max(
                                0.25 *
                                    config.voxelWidth,
                                1.0e-3);

                        constexpr int
                            maxSamplesPerPiece =
                                4096;

                        for (int pieceId = 0;
                             pieceId <
                                 routeMincoGuide
                                     .getPieceNum();
                             ++pieceId)
                        {
                            const auto &piece =
                                routeMincoGuide[
                                    pieceId];

                            const double duration =
                                piece.getDuration();

                            const double pieceMaxVel =
                                piece.getMaxVelRate();

                            const double pieceMaxAcc =
                                piece.getMaxAccRate();

                            if (!std::isfinite(
                                    pieceMaxVel) ||
                                !std::isfinite(
                                    pieceMaxAcc))
                            {
                                routeMincoGuideValid =
                                    false;

                                break;
                            }

                            guideMaxVel =
                                std::max(
                                    guideMaxVel,
                                    pieceMaxVel);

                            guideMaxAcc =
                                std::max(
                                    guideMaxAcc,
                                    pieceMaxAcc);

                            int sampleCount =
                                std::max(
                                    8,
                                    static_cast<int>(
                                        std::ceil(
                                            duration *
                                            std::max(
                                                pieceMaxVel,
                                                1.0e-3) /
                                            spatialSampleStep)));

                            if (sampleCount >
                                maxSamplesPerPiece)
                            {
                                sampleCount =
                                    maxSamplesPerPiece;

                                ++guideSamplingCapHits;
                            }

                            bool pieceColliding =
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

                                const double localTime =
                                    alpha *
                                    duration;

                                const Eigen::Vector3d pos =
                                    piece.getPos(
                                        localTime);

                                const Eigen::Vector3d vel =
                                    piece.getVel(
                                        localTime);

                                const Eigen::Vector3d acc =
                                    piece.getAcc(
                                        localTime);

                                const Eigen::Vector3d jer =
                                    piece.getJer(
                                        localTime);

                                ++guideTotalSamples;

                                if (voxelMap.query(
                                        pos) !=
                                    0)
                                {
                                    ++guideCollisionSamples;

                                    pieceColliding =
                                        true;
                                }

                                double thrust =
                                    0.0;

                                Eigen::Vector4d quat =
                                    Eigen::Vector4d::Zero();

                                Eigen::Vector3d bodyRate =
                                    Eigen::Vector3d::Zero();

                                guideFlatness.forward(
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
                                    guideFlatnessValid =
                                        false;

                                    continue;
                                }

                                guideMaxBodyRate =
                                    std::max(
                                        guideMaxBodyRate,
                                        bodyRate.norm());

                                const double tiltSinHalf =
                                    std::min(
                                        1.0,
                                        std::sqrt(
                                            std::max(
                                                0.0,
                                                quat(1) *
                                                    quat(1) +
                                                quat(2) *
                                                    quat(2))));

                                const double tilt =
                                    2.0 *
                                    std::asin(
                                        tiltSinHalf);

                                guideMaxTilt =
                                    std::max(
                                        guideMaxTilt,
                                        tilt);

                                guideMinThrust =
                                    std::min(
                                        guideMinThrust,
                                        thrust);

                                guideMaxThrust =
                                    std::max(
                                        guideMaxThrust,
                                        thrust);
                            }

                            if (pieceColliding)
                            {
                                ++guideCollisionPieces;
                            }
                        }
                    }

                    const double guideDiagnosticMs =
                        std::chrono::duration<
                            double,
                            std::milli>(
                                std::chrono::steady_clock::now() -
                                guideDiagnosticStarted)
                            .count();

                    if (!std::isfinite(
                            guideMinThrust))
                    {
                        guideMinThrust =
                            0.0;
                    }

                    if (!std::isfinite(
                            guideMaxThrust))
                    {
                        guideMaxThrust =
                            0.0;
                    }

                    const double guideCollisionRatio =
                        guideTotalSamples > 0
                            ? static_cast<double>(
                                  guideCollisionSamples) /
                                  static_cast<double>(
                                      guideTotalSamples)
                            : 0.0;

                    ROS_INFO_STREAM(
                        "TF_ROUTE_MINCO_GUIDE "
                        << "success="
                        << routeMincoGuideValid

                        << " route_points="
                        << route.size()

                        << " pieces="
                        << routeMincoGuide
                               .getPieceNum()

                        << " route_length="
                        << routeLength

                        << " ref_speed="
                        << guideReferenceSpeed

                        << " duration="
                        << (routeMincoGuideValid
                                ? routeMincoGuide
                                      .getTotalDuration()
                                : 0.0)

                        << " min_piece_time="
                        << minGuideTime

                        << " max_piece_time="
                        << maxGuideTime

                        << " energy="
                        << guideEnergy

                        << " build_ms="
                        << routeMincoGuideBuildMs

                        << " diagnostic_ms="
                        << guideDiagnosticMs

                        << " max_vel="
                        << guideMaxVel

                        << " max_acc="
                        << guideMaxAcc

                        << " max_body_rate="
                        << guideMaxBodyRate

                        << " max_tilt="
                        << guideMaxTilt

                        << " min_thrust="
                        << guideMinThrust

                        << " max_thrust="
                        << guideMaxThrust

                        << " flatness_valid="
                        << guideFlatnessValid

                        << " total_samples="
                        << guideTotalSamples

                        << " collision_samples="
                        << guideCollisionSamples

                        << " collision_ratio="
                        << guideCollisionRatio

                        << " collision_pieces="
                        << guideCollisionPieces

                        << " sample_cap_hits="
                        << guideSamplingCapHits);
                }
                else if (benchmarkProposedMode)
                {
                    // Cheap benchmark summary only.
                    ROS_INFO_STREAM(
                        "TF_ROUTE_MINCO_GUIDE "
                        << "success="
                        << routeMincoGuideValid

                        << " route_points="
                        << route.size()

                        << " pieces="
                        << routeMincoGuide.getPieceNum()

                        << " route_length="
                        << routeLength

                        << " ref_speed="
                        << guideReferenceSpeed

                        << " duration="
                        << (routeMincoGuideValid
                                ? routeMincoGuide
                                      .getTotalDuration()
                                : 0.0)

                        << " min_piece_time="
                        << minGuideTime

                        << " max_piece_time="
                        << maxGuideTime

                        << " energy="
                        << guideEnergy

                        << " build_ms="
                        << routeMincoGuideBuildMs

                        << " diagnostics_run=0");
                }

                if (legacyDebugMode)
                {
                    Trajectory<5>
                        repairedRouteMincoGuide;

                    std::vector<Eigen::Vector3d>
                        repairedGuideWaypoints;

                    traj_relevant::
                        RouteMincoGuideOptions
                            repairOptions;

                    repairOptions.reference_speed =
                        std::max(
                            0.5 *
                                config.maxVelMag,
                            1.0e-3);
                        
                    repairOptions.spatial_sample_step =
                        std::max(
                            0.25 *
                                config.voxelWidth,
                            1.0e-3);
                        
                    repairOptions.max_refinement_rounds =
                        4;
                        
                    traj_relevant::
                        RouteMincoGuideDiagnostics
                            repairDiagnostics;
                        
                    const bool repairedGuideSuccess =
                        traj_relevant::
                            buildCollisionAwareRouteMincoGuide(
                                route,
                                voxelMap,
                                repairOptions,
                                repairedRouteMincoGuide,
                                repairedGuideWaypoints,
                                &repairDiagnostics);
                            
                // ============================================================
                // Fixed-time MINCO waypoint-affinity validation.
                //
                // This validates the mathematical foundation of the future
                // continuous-time hard-SFC backend:
                //
                //     p_i(tau; P)
                //       = c_i(tau)
                //       + sum_j beta_ij(tau) P_j.
                //
                // No corridor or optimizer behavior is changed here.
                // ============================================================
                traj_relevant::
                    MincoWaypointAffineMap
                        guideAffineMap;
                                            
                traj_relevant::
                    MincoAffineValidationResult
                        guideAffineValidation;
                                            
                bool guideAffineBuildSuccess =
                    false;
                                            
                double guideAffineBuildMs =
                    0.0;
                                            
                if (config.experimentTag ==
                        "debug_metric" &&
                    routeMincoGuideValid)
                {
                    const auto affineBuildStarted =
                        std::chrono::
                            steady_clock::now();
                
                    guideAffineBuildSuccess =
                        guideAffineMap.build(
                            routeMincoGuideHeadPVA,
                            routeMincoGuideTailPVA,
                            routeMincoGuideTimes);
                        
                    guideAffineBuildMs =
                        std::chrono::duration<
                            double,
                            std::milli>(
                                std::chrono::
                                    steady_clock::now() -
                                affineBuildStarted)
                            .count();
                            
                    if (guideAffineBuildSuccess)
                    {
                        guideAffineValidation =
                            traj_relevant::
                                validateMincoWaypointAffineMap(
                                    guideAffineMap,
                                    routeMincoGuideHeadPVA,
                                    routeMincoGuideTailPVA,
                                    routeMincoGuideTimes,
                                    routeMincoGuideInnerPoints,
                                    16,
                                    12,
                                    0.5);
                    }
                
                    ROS_INFO_STREAM(
                        "TF_MINCO_AFFINE_MAP "
                        << "success="
                        << (guideAffineBuildSuccess &&
                            guideAffineValidation.valid)
                        
                        << " build_success="
                        << guideAffineBuildSuccess
                        
                        << " pieces="
                        << guideAffineMap
                               .pieceCount()
                        
                        << " inner_waypoints="
                        << guideAffineMap
                               .waypointCount()
                        
                        << " variable_dim="
                        << guideAffineMap
                               .variableDimension()
                        
                        << " test_trajectories="
                        << guideAffineValidation
                               .test_trajectory_count
                        
                        << " position_tests="
                        << guideAffineValidation
                               .position_test_count
                        
                        << " max_position_error_m="
                        << guideAffineValidation
                               .max_position_error_m
                        
                        << " max_relative_error="
                        << guideAffineValidation
                               .max_relative_error
                        
                        << " map_build_ms="
                        << guideAffineBuildMs
                        
                        << " validation_ms="
                        << guideAffineValidation
                               .validation_ms);
                }

                    ROS_INFO_STREAM(
                        "TF_ROUTE_MINCO_REFINE "
                        << "success="
                        << repairedGuideSuccess
                    
                        << " collision_free="
                        << repairDiagnostics
                               .collision_free
                    
                        << " initial_waypoints="
                        << repairDiagnostics
                               .initial_waypoint_count
                    
                        << " final_waypoints="
                        << repairDiagnostics
                               .final_waypoint_count
                    
                        << " initial_pieces="
                        << repairDiagnostics
                               .initial_piece_count
                    
                        << " final_pieces="
                        << repairDiagnostics
                               .final_piece_count
                    
                        << " rounds="
                        << repairDiagnostics
                               .refinement_rounds
                    
                        << " inserted="
                        << repairDiagnostics
                               .inserted_waypoint_count
                    
                        << " initial_collision_pieces="
                        << repairDiagnostics
                               .initial_collision_piece_count
                    
                        << " final_collision_pieces="
                        << repairDiagnostics
                               .final_collision_piece_count
                    
                        << " initial_collision_samples="
                        << repairDiagnostics
                               .initial_collision_sample_count
                    
                        << " final_collision_samples="
                        << repairDiagnostics
                               .final_collision_sample_count
                    
                        << " final_samples="
                        << repairDiagnostics
                               .final_total_samples
                    
                        << " build_ms="
                        << repairDiagnostics
                               .build_ms
                    
                        << " check_ms="
                        << repairDiagnostics
                               .collision_check_ms
                    
                        << " max_vel="
                        << repairDiagnostics
                               .final_max_vel
                    
                        << " max_acc="
                        << repairDiagnostics
                               .final_max_acc
                    
                        << " sample_cap_hits="
                        << repairDiagnostics
                               .sample_cap_hits);
                }
            }
            
            // ============================================================
            // Shared trajectory/backend context.
            //
            // These quantities depend only on the benchmark case and
            // physical/optimization configuration.  They do NOT depend
            // on how the safe-flight corridor is constructed.
            //
            // Keeping them outside the baseline corridor block is
            // necessary for the later method-isolated benchmark paths:
            //
            //   FIRI     -> common backend
            //   Proposed -> common backend
            // ============================================================
            Eigen::Matrix3d iniState;
            Eigen::Matrix3d finState;

            iniState <<
                route.front(),
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero();

            finState <<
                route.back(),
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero();

            // magnitudeBounds =
            // [v_max, omega_max, theta_max,
            //  thrust_min, thrust_max]^T
            Eigen::VectorXd magnitudeBounds(5);

            magnitudeBounds(0) =
                config.maxVelMag;

            magnitudeBounds(1) =
                config.maxBdrMag;

            magnitudeBounds(2) =
                config.maxTiltAngle;

            magnitudeBounds(3) =
                config.minThrust;

            magnitudeBounds(4) =
                config.maxThrust;

            // penaltyWeights =
            // [position, velocity, body-rate, tilt, thrust]^T
            Eigen::VectorXd penaltyWeights(5);

            penaltyWeights(0) =
                config.chiVec[0];

            penaltyWeights(1) =
                config.chiVec[1];

            penaltyWeights(2) =
                config.chiVec[2];

            penaltyWeights(3) =
                config.chiVec[3];

            penaltyWeights(4) =
                config.chiVec[4];

            // physicalParams =
            // [mass, gravity, horizontal drag, vertical drag,
            //  parasitic drag, speed smoothing]^T
            Eigen::VectorXd physicalParams(6);

            physicalParams(0) =
                config.vehicleMass;

            physicalParams(1) =
                config.gravAcc;

            physicalParams(2) =
                config.horizDrag;

            physicalParams(3) =
                config.vertDrag;

            physicalParams(4) =
                config.parasDrag;

            physicalParams(5) =
                config.speedEps;

            const int quadratureRes =
                config.integralIntervs;

                struct BackendAbResult
                {
                    bool setup_success =
                        false;
                
                    bool optimize_success =
                        false;
                    bool optimized_state_ready =
                        false;
                    Eigen::Matrix3Xd optimized_points;
                    Eigen::VectorXd optimized_times;
                
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
                    bool exact_mapping_valid =
                        false;
                    bool exact_certificate_valid =
                        false;
                    bool exact_contained =
                        false;
                    int exact_checked_faces =
                        0;
                    int exact_worst_piece =
                        -1;
                    int exact_worst_face =
                        -1;
                    double exact_max_violation_m =
                        -std::numeric_limits<double>::
                            infinity();
                    double exact_min_margin_m =
                        std::numeric_limits<double>::
                            infinity();
                    double exact_worst_tau =
                        0.0;
                    double exact_worst_t =
                        0.0;
                    double exact_certificate_ms =
                        0.0;
                };
                auto runBackendAb =
                    [&](const std::vector<Eigen::MatrixX4d> &corridors,
                        const double corridorPenaltyScale = 1.0)
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
                
                    Eigen::VectorXd backendPenaltyWeights = penaltyWeights;
                    if (backendPenaltyWeights.size() > 0)
                    {
                        backendPenaltyWeights(0) *= corridorPenaltyScale;
                    }
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
                            backendPenaltyWeights,   // 使用调整后的权重
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
                        
                        const Eigen::Matrix3Xd &optimizedPoints =
                            backendOptimizer.getOptimizedPoints();
                        const Eigen::VectorXd &optimizedTimes =
                            backendOptimizer.getOptimizedTimes();
                        const int expectedInnerPointCount =
                            std::max(result.trajectory_pieces - 1, 0);
                        result.optimized_state_ready =
                            optimizedPoints.rows() == 3 &&
                            optimizedPoints.cols() == expectedInnerPointCount &&
                            optimizedTimes.size() == result.trajectory_pieces &&
                            optimizedPoints.allFinite() &&
                            optimizedTimes.allFinite();
                        if (result.optimized_state_ready)
                        {
                            result.optimized_points = optimizedPoints;
                            result.optimized_times = optimizedTimes;
                        }
                        // ========================================================
                        // Exact continuous-time corridor certificate.
                        //
                        // In the current backend experiment:
                        //
                        //     lengthPerPiece = infinity
                        //
                        // and we require one trajectory piece per corridor before
                        // using the direct piece-id <-> corridor-id certificate.
                        // ========================================================
                        if (backendTrajectory.getPieceNum() ==
                            static_cast<int>(
                                corridors.size()))
                        {
                            result.exact_mapping_valid =
                                true;
                            const auto certificateStarted =
                                std::chrono::steady_clock::now();
                            bool allValid =
                                true;
                            bool allContained =
                                true;
                            for (int pieceId = 0;
                                 pieceId <
                                     backendTrajectory
                                         .getPieceNum();
                                 ++pieceId)
                            {
                                const auto certificate =
                                    traj_relevant::
                                        certifyMincoPieceInPolytope(
                                            backendTrajectory[
                                                pieceId],
                                            corridors[
                                                pieceId],
                                            1.0e-6,
                                            1.0e-10,
                                            1.0e-12);
                                result.exact_checked_faces +=
                                    certificate
                                        .checked_face_count;
                                if (!certificate.valid)
                                {
                                    allValid =
                                        false;
                                    allContained =
                                        false;
                                    continue;
                                }
                                if (!certificate.contained)
                                {
                                    allContained =
                                        false;
                                }
                                if (certificate
                                        .max_signed_violation_m >
                                    result
                                        .exact_max_violation_m)
                                {
                                    result.exact_max_violation_m =
                                        certificate
                                            .max_signed_violation_m;
                                    result.exact_min_margin_m =
                                        certificate
                                            .min_margin_m;
                                    result.exact_worst_piece =
                                        pieceId;
                                    result.exact_worst_face =
                                        certificate
                                            .worst_face;
                                    result.exact_worst_tau =
                                        certificate
                                            .worst_normalized_time;
                                    result.exact_worst_t =
                                        certificate
                                            .worst_physical_time;
                                }
                            }
                            result.exact_certificate_valid =
                                allValid;
                            result.exact_contained =
                                allValid &&
                                allContained;
                            result.exact_certificate_ms =
                                std::chrono::duration<
                                    double,
                                    std::milli>(
                                        std::chrono::
                                            steady_clock::now() -
                                        certificateStarted)
                                    .count();
                        }
                    }
                
                    return result;
                };
            
                gcopter::GCOPTER_PolytopeSFC
                    guideMetricEvaluator;

                gcopter::GCOPTER_PolytopeSFC::
                    GaussNewtonDeformationMetrics
                        guideMetrics;

                bool guideMetricStateReady =
                    false;

                bool guideMetricSuccess =
                    false;

                double guideMetricMs =
                    0.0;

                if (runProposedCore &&
                    routeMincoGuideValid)
                {
                    guideMetricStateReady =
                        guideMetricEvaluator
                            .setGaussNewtonReferenceState(
                                routeMincoGuideHeadPVA,
                                routeMincoGuideTailPVA,
                                routeMincoGuideInnerPoints,
                                routeMincoGuideTimes,
                                quadratureRes,
                                magnitudeBounds,
                                physicalParams);

                    if (guideMetricStateReady)
                    {
                        const auto guideMetricStarted =
                            std::chrono::steady_clock::now();

                        guideMetricSuccess =
                            guideMetricEvaluator
                                .computeGaussNewtonDeformationMetrics(
                                    guideMetrics,
                                    0.01,
                                    1.0e-3,
                                    4.0,
                                    10.0);

                        guideMetricMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::steady_clock::now() -
                                    guideMetricStarted)
                                .count();
                    }

                    int validGuideMetrics =
                        0;

                    int anisotropicGuideMetrics =
                        0;

                    double meanGuideAnisotropy =
                        0.0;

                    double maxGuideAnisotropy =
                        0.0;

                    for (int pieceId = 0;
                         pieceId <
                             static_cast<int>(
                                 guideMetrics.size());
                         ++pieceId)
                    {
                        const auto &metric =
                            guideMetrics[
                                pieceId];

                        if (!metric.valid)
                        {
                            continue;
                        }

                        ++validGuideMetrics;

                        meanGuideAnisotropy +=
                            metric.corridorAnisotropy;

                        maxGuideAnisotropy =
                            std::max(
                                maxGuideAnisotropy,
                                metric.corridorAnisotropy);

                        if (metric.corridorAnisotropy >
                            1.0 + 1.0e-3)
                        {
                            ++anisotropicGuideMetrics;
                        }

                        if (legacyDebugMode)
                        {
                            ROS_INFO_STREAM(
                                "TF_ROUTE_MINCO_CSGN_PIECE "
                                << "piece="
                                << pieceId

                                << " valid="
                                << metric.valid

                                << " anisotropy="
                                << metric.corridorAnisotropy

                                << " principal_gap="
                                << metric.principalGap

                                << " dir_x="
                                << metric.principalDirection.x()

                                << " dir_y="
                                << metric.principalDirection.y()

                                << " dir_z="
                                << metric.principalDirection.z());
                        }
                    }

                    if (validGuideMetrics > 0)
                    {
                        meanGuideAnisotropy /=
                            static_cast<double>(
                                validGuideMetrics);
                    }

                    ROS_INFO_STREAM(
                        "TF_ROUTE_MINCO_CSGN "
                        << "success="
                        << guideMetricSuccess

                        << " state_ready="
                        << guideMetricStateReady

                        << " guide_pieces="
                        << routeMincoGuide.getPieceNum()

                        << " metric_count="
                        << guideMetrics.size()

                        << " valid="
                        << validGuideMetrics

                        << " anisotropic="
                        << anisotropicGuideMetrics

                        << " mean_anisotropy="
                        << meanGuideAnisotropy

                        << " max_anisotropy="
                        << maxGuideAnisotropy

                        << " metric_ms="
                        << guideMetricMs);
                }

            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);
            record.map_point_count = static_cast<int>(pc.size());

            // ============================================================
            // Proposed route-segment metric mapping + Active-Witness SFC.
            //
            // This stage depends only on:
            //   route
            //   direct MINCO guide / guide CSGN
            //   obstacle surface cloud
            //   map bounds
            //
            // It does NOT depend on baseline FIRI or baseline GCOPTER.
            // ============================================================

            const int guideRawSegmentCount =
                runProposedCore
                    ? std::max(
                          0,
                          static_cast<int>(
                              route.size()) -
                              1)
                    : 0;
                        
            sfc_gen::SegmentDeformationMetrics
                guideSegmentMetrics;
                        
            guideSegmentMetrics.resize(
                guideRawSegmentCount);
            
            const bool guideMetricCardinalityValid =
                runProposedCore &&
                guideMetricSuccess &&
                routeMincoGuideValid &&
                routeMincoGuide.getPieceNum() ==
                    guideRawSegmentCount &&
                static_cast<int>(
                    guideMetrics.size()) ==
                    guideRawSegmentCount;
                
            int guideSegmentMetricValidCount =
                0;
                
            for (int segmentId = 0;
                 segmentId <
                     guideRawSegmentCount;
                 ++segmentId)
            {
                auto &segmentMetric =
                    guideSegmentMetrics[
                        segmentId];
                    
                segmentMetric.source_piece_id =
                    segmentId;
                    
                // Exact one-to-one mapping:
                //
                // guide MINCO piece i
                //     <-> RRT edge i
                //     <-> corridor i
                segmentMetric.mapping_distance =
                    0.0;
                    
                segmentMetric.utility =
                    Eigen::Matrix3d::Identity();
                    
                segmentMetric.valid =
                    false;
                    
                if (!guideMetricCardinalityValid)
                {
                    continue;
                }
            
                const auto &guideMetric =
                    guideMetrics[
                        segmentId];
                    
                if (!guideMetric.valid ||
                    !guideMetric.corridorUtility
                         .allFinite())
                {
                    continue;
                }
            
                segmentMetric.utility =
                    guideMetric
                        .corridorUtility;
            
                segmentMetric.valid =
                    true;
            
                ++guideSegmentMetricValidCount;
            }

            const bool guideSegmentMetricsReady =
                guideMetricCardinalityValid &&
                guideSegmentMetricValidCount ==
                    guideRawSegmentCount;

            // ------------------------------------------------------------
            // Shared compact-corridor options.
            //
            // BATCH_SET_COVER remains the template because the legacy
            // batch ablation below still uses guideCompactOptions.
            //
            // The Proposed path copies it and changes only the candidate
            // selection mode to ACTIVE_WITNESS.
            // ------------------------------------------------------------
            traj_relevant::CompactCorridorOptions
                guideCompactOptions;

            guideCompactOptions.max_extra_radius =
                std::max(
                    config.tfFiriRange,
                    config.voxelWidth);
                
            guideCompactOptions.min_extra_ratio =
                0.25;
                
            guideCompactOptions.overlap_radius =
                0.01;
                
            guideCompactOptions.epsilon =
                1.0e-6;
                
            guideCompactOptions.candidate_selection_mode =
                traj_relevant::
                    CandidateSelectionMode::
                        BATCH_SET_COVER;
                
            // ------------------------------------------------------------
            // Formal Proposed corridor:
            // Direct-guide CSGN + Active-Witness.
            // ------------------------------------------------------------
            auto activeGuideOptions =
                guideCompactOptions;
                
            activeGuideOptions.candidate_selection_mode =
                traj_relevant::
                    CandidateSelectionMode::
                        ACTIVE_WITNESS;
                
            std::vector<Eigen::MatrixX4d>
                activeGuideHPolys;
                
            sfc_gen::TrajectoryRelevantCompactInfos
                activeGuideInfos;
                
            bool activeGuideSuccess =
                false;
                
            double activeGuideMs =
                0.0;
                
            if (guideSegmentMetricsReady)
            {
                const auto activeGuideStarted =
                    std::chrono::steady_clock::now();
            
                activeGuideSuccess =
                    sfc_gen::
                        trajectoryRelevantCompactCover(
                            route,
                            pc,
                            voxelMap.getOrigin(),
                            voxelMap.getCorner(),
                            std::numeric_limits<double>::
                                infinity(),
                            activeGuideOptions,
                            activeGuideHPolys,
                            activeGuideInfos,
                            &guideSegmentMetrics);
                        
                activeGuideMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::
                                steady_clock::now() -
                            activeGuideStarted)
                        .count();
            }

            // ------------------------------------------------------------
            // Proposed corridor diagnostics required by the paper logger.
            // ------------------------------------------------------------
            int activeGuideTotalFaces =
                0;

            int activeGuideDomainFaces =
                0;

            int activeGuideObstacleFaces =
                0;

            int activeGuideCandidates =
                0;

            int activeGuideRounds =
                0;

            int activeGuideRedundancyRemoved =
                0;

            std::int64_t
                activeGuideWitnessTests =
                    0;

            std::int64_t
                activeGuideFaceTests =
                    0;

            int activeGuideSafetyCount =
                0;

            for (const auto &info :
                 activeGuideInfos)
            {
                activeGuideTotalFaces +=
                    info.total_face_count;
            
                activeGuideDomainFaces +=
                    info.domain_face_count;
            
                activeGuideObstacleFaces +=
                    info.selected_obstacle_face_count;
            
                activeGuideCandidates +=
                    info.candidate_count;
            
                activeGuideRounds +=
                    info.active_witness_rounds;
            
                activeGuideRedundancyRemoved +=
                    info.redundancy_removed;
            
                activeGuideWitnessTests +=
                    info.witness_distance_tests;
            
                activeGuideFaceTests +=
                    info.obstacle_face_tests;
            
                activeGuideSafetyCount +=
                    info.safety_verified
                        ? 1
                        : 0;
            }

            // ------------------------------------------------------------
            // Explicit neighboring-corridor overlap verification.
            // ------------------------------------------------------------
            int activeGuideAdjacentOverlapValidCount =
                0;

            for (int corridorId = 1;
                 corridorId <
                     static_cast<int>(
                         activeGuideHPolys.size());
                 ++corridorId)
            {
                if (geo_utils::overlap(
                        activeGuideHPolys[
                            corridorId - 1],
                        activeGuideHPolys[
                            corridorId],
                        0.01))
                {
                    ++activeGuideAdjacentOverlapValidCount;
                }
            }

            const int activeGuideAdjacentOverlapCount =
                std::max(
                    0,
                    static_cast<int>(
                        activeGuideHPolys.size()) -
                        1);

            // ============================================================
            // Proposed backend + exact hard SFC closure.
            //
            // IMPORTANT:
            // This computation depends only on:
            //
            //   activeGuideHPolys
            //   shared backend context
            //   runBackendAb()
            //
            // It does NOT depend on baseline FIRI or baseline GCOPTER.
            // ============================================================

            BackendAbResult
                activeGuideBackendResult;

            if (activeGuideSuccess)
            {
                activeGuideBackendResult =
                    runBackendAb(
                        activeGuideHPolys);
            }

            // ------------------------------------------------------------
            // Exact continuous-time hard SFC closure.
            //
            // The projection consumes the optimized points/times from the
            // SAME Active-Witness GCOPTER backend solve.
            // ------------------------------------------------------------
            bool hardProjectionSourceReady =
                activeGuideSuccess &&
                activeGuideBackendResult
                    .setup_success &&
                activeGuideBackendResult
                    .optimize_success &&
                activeGuideBackendResult
                    .optimized_state_ready;

            traj_relevant::
                ExactSfcProjectionResult
                    hardProjectionResult;

            Trajectory<5>
                hardProjectedTrajectory;

            Eigen::Matrix3Xd
                hardProjectedPoints;

            if (hardProjectionSourceReady)
            {
                // ========================================================
                // Bernstein hard-SFC assembly diagnostic.
                //
                // Read-only:
                //   - reuse existing Active-Witness SFC;
                //   - reuse existing optimized soft MINCO state;
                //   - do not solve a Bernstein QP;
                //   - do not modify the trajectory.
                // ========================================================
                const auto bernsteinDiagStarted =
                    std::chrono::steady_clock::now();

                traj_relevant::
                    MincoWaypointAffineMap
                        bernsteinAffineMap;

                const auto bernsteinAffineStarted =
                    std::chrono::steady_clock::now();

                const bool bernsteinAffineValid =
                    bernsteinAffineMap.build(
                        iniState,
                        finState,
                        activeGuideBackendResult
                            .optimized_times);

                const double bernsteinAffineBuildMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::
                                steady_clock::now() -
                            bernsteinAffineStarted)
                        .count();

                traj_relevant::
                    BernsteinSfcConstraintSet
                        bernsteinSet;

                bool bernsteinSoftPointValid =
                    false;

                bool bernsteinSoftFeasible =
                    false;

                int bernsteinMatrixViolated =
                    0;

                int bernsteinControlFaceViolated =
                    0;

                double maxBernsteinNormalizedResidual =
                    -std::numeric_limits<double>::
                        infinity();

                double maxBernsteinControlViolationM =
                    -std::numeric_limits<double>::
                        infinity();

                if (bernsteinAffineValid)
                {
                    bernsteinSet =
                        traj_relevant::
                            buildBernsteinSfcConstraintSet(
                                bernsteinAffineMap,
                                activeGuideHPolys);

                    if (bernsteinSet.valid &&
                        !bernsteinSet.fixed_infeasible)
                    {
                        const Eigen::VectorXd z0 =
                            traj_relevant::
                                flattenMincoWaypoints(
                                    activeGuideBackendResult
                                        .optimized_points);

                        bernsteinSoftPointValid =
                            z0.size() ==
                                bernsteinSet
                                    .variable_dimension &&
                            z0.allFinite();

                        if (bernsteinSoftPointValid)
                        {
                            const Eigen::VectorXd residual =
                                bernsteinSet.A * z0 -
                                bernsteinSet.b;

                            if (residual.size() > 0 &&
                                residual.allFinite())
                            {
                                maxBernsteinNormalizedResidual =
                                    residual.maxCoeff();

                                for (int rowId = 0;
                                     rowId < residual.size();
                                     ++rowId)
                                {
                                    if (residual(rowId) >
                                        1.0e-10)
                                    {
                                        ++bernsteinMatrixViolated;
                                    }
                                }
                            }
                        }

                        // -----------------------------------------------
                        // Independent physical-space Bernstein check.
                        //
                        // Signed distance:
                        //
                        //     (n^T C_k + d) / ||n||
                        //
                        // is measured in meters.
                        // -----------------------------------------------
                        for (int pieceId = 0;
                             pieceId <
                                 bernsteinAffineMap
                                     .pieceCount();
                             ++pieceId)
                        {
                            const auto &poly =
                                activeGuideHPolys[
                                    pieceId];

                            for (int controlId = 0;
                                 controlId < 6;
                                 ++controlId)
                            {
                                Eigen::Vector3d
                                    controlOffset;

                                Eigen::VectorXd
                                    controlBeta;

                                if (!bernsteinAffineMap
                                         .bernsteinControlAffineCoefficients(
                                             pieceId,
                                             controlId,
                                             controlOffset,
                                             controlBeta))
                                {
                                    continue;
                                }

                                Eigen::Vector3d controlPoint =
                                    controlOffset;

                                for (int waypointId = 0;
                                     waypointId <
                                         controlBeta.size();
                                     ++waypointId)
                                {
                                    controlPoint +=
                                        controlBeta(
                                            waypointId) *
                                        activeGuideBackendResult
                                            .optimized_points
                                            .col(
                                                waypointId);
                                }

                                for (int faceId = 0;
                                     faceId <
                                         poly.rows();
                                     ++faceId)
                                {
                                    const Eigen::Vector3d normal =
                                        poly.block<1, 3>(
                                                faceId,
                                                0)
                                            .transpose();

                                    const double normalNorm =
                                        normal.norm();

                                    if (!std::isfinite(
                                            normalNorm) ||
                                        normalNorm <=
                                            1.0e-12)
                                    {
                                        continue;
                                    }

                                    const double
                                        signedDistanceM =
                                            (normal.dot(
                                                 controlPoint) +
                                             poly(
                                                 faceId,
                                                 3)) /
                                            normalNorm;

                                    maxBernsteinControlViolationM =
                                        std::max(
                                            maxBernsteinControlViolationM,
                                            signedDistanceM);

                                    if (signedDistanceM >
                                        1.0e-6)
                                    {
                                        ++bernsteinControlFaceViolated;
                                    }
                                }
                            }
                        }

                        bernsteinSoftFeasible =
                            bernsteinSoftPointValid &&
                            std::isfinite(
                                maxBernsteinControlViolationM) &&
                            maxBernsteinControlViolationM <=
                                1.0e-6;
                    }
                }

                // Bernstein convex-hull containment is sufficient
                // for continuous-time containment. Therefore this
                // condition must never occur.
                const bool bernsteinImplicationMismatch =
                    bernsteinSoftFeasible &&
                    activeGuideBackendResult
                        .exact_certificate_valid &&
                    !activeGuideBackendResult
                         .exact_contained;

                const double bernsteinDiagTotalMs =
                    std::chrono::duration<
                        double,
                        std::milli>(
                            std::chrono::
                                steady_clock::now() -
                            bernsteinDiagStarted)
                        .count();

                ROS_INFO_STREAM(
                    "TF_BERNSTEIN_SFC_DIAG "
                    << "affine_valid="
                    << bernsteinAffineValid
                    << " constraint_valid="
                    << bernsteinSet.valid
                    << " fixed_infeasible="
                    << bernsteinSet.fixed_infeasible
                    << " soft_point_valid="
                    << bernsteinSoftPointValid
                    << " soft_feasible="
                    << bernsteinSoftFeasible
                    << " variables="
                    << bernsteinSet.variable_dimension
                    << " constraints="
                    << bernsteinSet.constraint_count
                    << " skipped_fixed="
                    << bernsteinSet
                           .skipped_fixed_constraints
                    << " matrix_violated="
                    << bernsteinMatrixViolated
                    << " control_face_violated="
                    << bernsteinControlFaceViolated
                    << " max_normalized_residual="
                    << maxBernsteinNormalizedResidual
                    << " max_control_violation_m="
                    << maxBernsteinControlViolationM
                    << " affine_build_ms="
                    << bernsteinAffineBuildMs
                    << " assembly_ms="
                    << bernsteinSet.assembly_ms
                    << " diag_total_ms="
                    << bernsteinDiagTotalMs
                    << " exact_cert_valid="
                    << activeGuideBackendResult
                           .exact_certificate_valid
                    << " exact_contained="
                    << activeGuideBackendResult
                           .exact_contained
                    << " exact_violation_m="
                    << activeGuideBackendResult
                           .exact_max_violation_m
                    << " implication_mismatch="
                    << bernsteinImplicationMismatch);

                for (int subdivisionDepth = 0;
                     subdivisionDepth <= 3;
                     ++subdivisionDepth)
                {
                    const auto subdividedSet =
                        traj_relevant::
                            buildSubdividedBernsteinSfcConstraintSet(
                                bernsteinAffineMap,
                                activeGuideHPolys,
                                subdivisionDepth);
                            
                    double maxResidual =
                        -std::numeric_limits<double>::
                            infinity();
                            
                    int violated =
                        0;
                            
                    if (subdividedSet.valid &&
                        !subdividedSet.fixed_infeasible)
                    {
                        const Eigen::VectorXd z0 =
                            traj_relevant::
                                flattenMincoWaypoints(
                                    activeGuideBackendResult
                                        .optimized_points);
                                
                        const Eigen::VectorXd residual =
                            subdividedSet.A * z0 -
                            subdividedSet.b;
                                
                        if (residual.size() > 0 &&
                            residual.allFinite())
                        {
                            maxResidual =
                                residual.maxCoeff();
                        
                            for (int rowId = 0;
                                 rowId <
                                     residual.size();
                                 ++rowId)
                            {
                                if (residual(rowId) >
                                    1.0e-10)
                                {
                                    ++violated;
                                }
                            }
                        }
                    }
                
                    ROS_INFO_STREAM(
                        "TF_BERNSTEIN_SUBDIV_DIAG "
                        << "depth="
                        << subdivisionDepth
                        << " valid="
                        << subdividedSet.valid
                        << " fixed_infeasible="
                        << subdividedSet.fixed_infeasible
                        << " constraints="
                        << subdividedSet.constraint_count
                        << " violated="
                        << violated
                        << " max_normalized_residual="
                        << maxResidual
                        << " assembly_ms="
                        << subdividedSet.assembly_ms);
                }

                // ========================================================
                // Local Bernstein cut diagnostic.
                //
                // Uses ONLY the exact worst witness:
                //
                //   (piece*, face*, tau*)
                //
                // and constructs one dyadic leaf for that witness.
                //
                // Constraint count must remain <= 6 regardless of depth.
                // ========================================================
                if (bernsteinAffineValid &&
                    activeGuideBackendResult
                        .exact_certificate_valid &&
                    !activeGuideBackendResult
                         .exact_contained &&
                    activeGuideBackendResult
                        .exact_worst_piece >= 0 &&
                    activeGuideBackendResult
                        .exact_worst_face >= 0)
                {
                    const Eigen::VectorXd z0 =
                        traj_relevant::
                            flattenMincoWaypoints(
                                activeGuideBackendResult
                                    .optimized_points);

                    double previousBoundM =
                        std::numeric_limits<double>::
                            infinity();

                    for (int localDepth = 0;
                         localDepth <= 8;
                         ++localDepth)
                    {
                        const auto localCut =
                            traj_relevant::
                                buildLocalBernsteinCut(
                                    bernsteinAffineMap,
                                    activeGuideHPolys,
                                    activeGuideBackendResult
                                        .exact_worst_piece,
                                    activeGuideBackendResult
                                        .exact_worst_face,
                                    activeGuideBackendResult
                                        .exact_worst_tau,
                                    localDepth);

                        double maxNormalizedResidual =
                            -std::numeric_limits<double>::
                                infinity();

                        if (localCut.valid &&
                            !localCut.fixed_infeasible &&
                            localCut.A.rows() > 0 &&
                            z0.size() ==
                                localCut.A.cols())
                        {
                            const Eigen::VectorXd residual =
                                localCut.A * z0 -
                                localCut.b;

                            if (residual.size() > 0 &&
                                residual.allFinite())
                            {
                                maxNormalizedResidual =
                                    residual.maxCoeff();
                            }
                        }

                        double boundM =
                            -std::numeric_limits<double>::
                                infinity();

                        int boundLeaf =
                            -1;

                        double boundIntervalBegin =
                            0.0;

                        double boundIntervalEnd =
                            1.0;

                        const bool boundValid =
                            traj_relevant::
                                evaluateLocalBernsteinFaceBoundM(
                                    bernsteinAffineMap,
                                    activeGuideHPolys,
                                    activeGuideBackendResult
                                        .optimized_points,
                                    activeGuideBackendResult
                                        .exact_worst_piece,
                                    activeGuideBackendResult
                                        .exact_worst_face,
                                    activeGuideBackendResult
                                        .exact_worst_tau,
                                    localDepth,
                                    boundM,
                                    boundLeaf,
                                    boundIntervalBegin,
                                    boundIntervalEnd);

                        const double boundGapM =
                            boundValid
                                ? boundM -
                                      activeGuideBackendResult
                                          .exact_max_violation_m
                                : std::numeric_limits<double>::
                                      infinity();

                        // For the SAME violating face and interval
                        // containing the exact maximizer, Bernstein
                        // must remain an upper bound.
                        const bool upperBoundMismatch =
                            boundValid &&
                            boundM + 1.0e-9 <
                                activeGuideBackendResult
                                    .exact_max_violation_m;

                        const bool nonMonotone =
                            boundValid &&
                            std::isfinite(
                                previousBoundM) &&
                            boundM >
                                previousBoundM +
                                    1.0e-9;

                        ROS_INFO_STREAM(
                            "TF_BERNSTEIN_LOCAL_DIAG "
                            << "depth="
                            << localDepth
                            << " valid="
                            << localCut.valid
                            << " fixed_infeasible="
                            << localCut.fixed_infeasible
                            << " piece="
                            << localCut.piece
                            << " face="
                            << localCut.face
                            << " tau="
                            << activeGuideBackendResult
                                   .exact_worst_tau
                            << " leaf="
                            << localCut.leaf
                            << " interval_begin="
                            << localCut.interval_begin
                            << " interval_end="
                            << localCut.interval_end
                            << " constraints="
                            << localCut.A.rows()
                            << " max_normalized_residual="
                            << maxNormalizedResidual
                            << " bound_valid="
                            << boundValid
                            << " bound_m="
                            << boundM
                            << " exact_violation_m="
                            << activeGuideBackendResult
                                   .exact_max_violation_m
                            << " bound_gap_m="
                            << boundGapM
                            << " upper_bound_mismatch="
                            << upperBoundMismatch
                            << " non_monotone="
                            << nonMonotone);

                        if (boundValid)
                        {
                            previousBoundM =
                                boundM;
                        }
                    }
                }

                // ========================================================
                // One-step Lazy Bernstein hard-projection diagnostic.
                //
                // IMPORTANT:
                //   - does NOT replace the existing Exact-Hard output;
                //   - does NOT modify the final benchmark trajectory;
                //   - uses only the current global exact worst witness;
                //   - adaptively refines ONE dyadic Bernstein leaf;
                //   - solves at most six local hard inequalities.
                // ========================================================
                if (bernsteinAffineValid &&
                    activeGuideBackendResult
                        .exact_certificate_valid &&
                    !activeGuideBackendResult
                         .exact_contained &&
                    activeGuideBackendResult
                        .exact_worst_piece >= 0 &&
                    activeGuideBackendResult
                        .exact_worst_face >= 0)
                {
                    const double
                        bernsteinClosureToleranceM =
                            1.0e-6;

                    const int
                        bernsteinMaxAdaptiveDepth =
                            16;

                    int selectedDepth =
                        -1;

                    double selectedBoundM =
                        std::numeric_limits<double>::
                            infinity();

                    double selectedBoundGapM =
                        std::numeric_limits<double>::
                            infinity();

                    for (int depth = 0;
                         depth <=
                             bernsteinMaxAdaptiveDepth;
                         ++depth)
                    {
                        double boundM =
                            -std::numeric_limits<double>::
                                infinity();

                        int leafId =
                            -1;

                        double intervalBegin =
                            0.0;

                        double intervalEnd =
                            1.0;

                        const bool boundValid =
                            traj_relevant::
                                evaluateLocalBernsteinFaceBoundM(
                                    bernsteinAffineMap,
                                    activeGuideHPolys,
                                    activeGuideBackendResult
                                        .optimized_points,
                                    activeGuideBackendResult
                                        .exact_worst_piece,
                                    activeGuideBackendResult
                                        .exact_worst_face,
                                    activeGuideBackendResult
                                        .exact_worst_tau,
                                    depth,
                                    boundM,
                                    leafId,
                                    intervalBegin,
                                    intervalEnd);

                        if (!boundValid)
                        {
                            break;
                        }

                        const double boundGapM =
                            boundM -
                            activeGuideBackendResult
                                .exact_max_violation_m;

                        selectedDepth =
                            depth;

                        selectedBoundM =
                            boundM;

                        selectedBoundGapM =
                            boundGapM;

                        if (boundGapM <=
                            bernsteinClosureToleranceM)
                        {
                            break;
                        }
                    }

                    const auto localCut =
                        traj_relevant::
                            buildLocalBernsteinCut(
                                bernsteinAffineMap,
                                activeGuideHPolys,
                                activeGuideBackendResult
                                    .exact_worst_piece,
                                activeGuideBackendResult
                                    .exact_worst_face,
                                activeGuideBackendResult
                                    .exact_worst_tau,
                                selectedDepth);

                    bool qpAttempted =
                        false;

                    bool qpSuccess =
                        false;

                    int qpSweeps =
                        0;

                    double qpMaxPrimal =
                        std::numeric_limits<double>::
                            infinity();

                    double qpMaxDualChange =
                        std::numeric_limits<double>::
                            infinity();

                    double qpMs =
                        0.0;

                    double correctionL2M =
                        std::numeric_limits<double>::
                            infinity();

                    double maxWaypointDispM =
                        std::numeric_limits<double>::
                            infinity();

                    bool rebuiltValid =
                        false;

                    bool afterCertValid =
                        false;

                    bool afterContained =
                        false;

                    double afterViolationM =
                        std::numeric_limits<double>::
                            infinity();

                    int afterWorstPiece =
                        -1;

                    int afterWorstFace =
                        -1;

                    double afterWorstTau =
                        0.0;

                    if (selectedDepth >= 0 &&
                        localCut.valid &&
                        !localCut.fixed_infeasible &&
                        localCut.A.rows() > 0 &&
                        localCut.A.rows() <= 6)
                    {
                        const Eigen::VectorXd z0 =
                            traj_relevant::
                                flattenMincoWaypoints(
                                    activeGuideBackendResult
                                        .optimized_points);

                        std::vector<Eigen::VectorXd>
                            qpRows;

                        std::vector<double>
                            qpRhs;

                        qpRows.reserve(
                            localCut.A.rows());

                        qpRhs.reserve(
                            localCut.A.rows());

                        for (int rowId = 0;
                             rowId <
                                 localCut.A.rows();
                             ++rowId)
                        {
                            qpRows.push_back(
                                localCut.A
                                    .row(rowId)
                                    .transpose());

                            qpRhs.push_back(
                                localCut.b(rowId));
                        }

                        qpAttempted =
                            true;

                        const auto qpStarted =
                            std::chrono::
                                steady_clock::now();

                        const auto qp =
                            traj_relevant::
                                solveEuclideanHalfspaceProjection(
                                    z0,
                                    qpRows,
                                    qpRhs,
                                    1.0e-10,
                                    1.0e-12,
                                    20000);

                        qpMs =
                            std::chrono::duration<
                                double,
                                std::milli>(
                                    std::chrono::
                                        steady_clock::now() -
                                    qpStarted)
                                .count();

                        qpSuccess =
                            qp.success &&
                            qp.solution.allFinite();

                        qpSweeps =
                            qp.sweeps;

                        qpMaxPrimal =
                            qp.max_primal_violation;

                        qpMaxDualChange =
                            qp.max_dual_change;

                        if (qpSuccess)
                        {
                            correctionL2M =
                                (qp.solution - z0)
                                    .norm();

                            Eigen::Matrix3Xd
                                diagnosticPoints;

                            if (traj_relevant::
                                    unflattenMincoWaypoints(
                                        qp.solution,
                                        diagnosticPoints))
                            {
                                maxWaypointDispM =
                                    0.0;

                                for (int waypointId = 0;
                                     waypointId <
                                         diagnosticPoints.cols();
                                     ++waypointId)
                                {
                                    maxWaypointDispM =
                                        std::max(
                                            maxWaypointDispM,
                                            (diagnosticPoints
                                                 .col(waypointId) -
                                             activeGuideBackendResult
                                                 .optimized_points
                                                 .col(waypointId))
                                                .norm());
                                }

                                minco::MINCO_S3NU
                                    diagnosticMinco;

                                diagnosticMinco
                                    .setConditions(
                                        iniState,
                                        finState,
                                        activeGuideBackendResult
                                            .optimized_times
                                            .size());

                                diagnosticMinco
                                    .setParameters(
                                        diagnosticPoints,
                                        activeGuideBackendResult
                                            .optimized_times);

                                Trajectory<5>
                                    diagnosticTrajectory;

                                diagnosticMinco
                                    .getTrajectory(
                                        diagnosticTrajectory);

                                rebuiltValid =
                                    diagnosticTrajectory
                                        .getPieceNum() ==
                                    static_cast<int>(
                                        activeGuideHPolys
                                            .size());

                                if (rebuiltValid)
                                {
                                    const auto afterCert =
                                        traj_relevant::
                                            certifyMincoTrajectoryInCorridors(
                                                diagnosticTrajectory,
                                                activeGuideHPolys,
                                                1.0e-6);

                                    afterCertValid =
                                        afterCert.valid;

                                    afterContained =
                                        afterCert.contained;

                                    afterViolationM =
                                        afterCert
                                            .worst
                                            .violation_m;

                                    afterWorstPiece =
                                        afterCert
                                            .worst
                                            .piece;

                                    afterWorstFace =
                                        afterCert
                                            .worst
                                            .face;

                                    afterWorstTau =
                                        afterCert
                                            .worst
                                            .normalized_time;
                                }
                            }
                        }
                    }

                    ROS_INFO_STREAM(
                        "TF_BERNSTEIN_ONE_STEP_QP "
                        << "selected_depth="
                        << selectedDepth
                        << " bound_m="
                        << selectedBoundM
                        << " bound_gap_m="
                        << selectedBoundGapM
                        << " piece="
                        << localCut.piece
                        << " face="
                        << localCut.face
                        << " leaf="
                        << localCut.leaf
                        << " interval_begin="
                        << localCut.interval_begin
                        << " interval_end="
                        << localCut.interval_end
                        << " constraints="
                        << localCut.A.rows()
                        << " qp_attempted="
                        << qpAttempted
                        << " qp_success="
                        << qpSuccess
                        << " qp_sweeps="
                        << qpSweeps
                        << " qp_max_primal="
                        << qpMaxPrimal
                        << " qp_max_dual_change="
                        << qpMaxDualChange
                        << " qp_ms="
                        << qpMs
                        << " correction_l2_m="
                        << correctionL2M
                        << " max_waypoint_disp_m="
                        << maxWaypointDispM
                        << " rebuilt_valid="
                        << rebuiltValid
                        << " before_violation_m="
                        << activeGuideBackendResult
                               .exact_max_violation_m
                        << " after_cert_valid="
                        << afterCertValid
                        << " after_contained="
                        << afterContained
                        << " after_violation_m="
                        << afterViolationM
                        << " after_worst_piece="
                        << afterWorstPiece
                        << " after_worst_face="
                        << afterWorstFace
                        << " after_worst_tau="
                        << afterWorstTau);
                }

                traj_relevant::
                    ExactSfcProjectionOptions
                        projectionOptions;
            
                projectionOptions
                    .containment_tolerance_m =
                        1.0e-6;
            
                Trajectory<5>
                    lazyBernsteinTrajectory;

                Eigen::Matrix3Xd
                    lazyBernsteinPoints;

                traj_relevant::
                    LazyBernsteinProjectionOptions
                        lazyBernsteinOptions;

                lazyBernsteinOptions
                    .containment_tolerance_m =
                        1.0e-6;

                lazyBernsteinOptions
                    .bound_gap_tolerance_m =
                        1.0e-6;

                const auto lazyBernsteinResult =
                    traj_relevant::
                        projectMincoToLazyBernsteinSfc(
                            iniState,
                            finState,
                            activeGuideBackendResult
                                .optimized_points,
                            activeGuideBackendResult
                                .optimized_times,
                            activeGuideHPolys,
                            lazyBernsteinTrajectory,
                            lazyBernsteinPoints,
                            lazyBernsteinOptions);
                        
                ROS_INFO_STREAM(
                    "TF_LAZY_BERNSTEIN_RESULT "
                    << "success="
                    << lazyBernsteinResult.success
                    << " initial_contained="
                    << lazyBernsteinResult.initial_contained
                    << " initial_violation_m="
                    << lazyBernsteinResult
                           .initial_max_violation_m
                    << " final_cert_valid="
                    << lazyBernsteinResult
                           .final_certificate_valid
                    << " final_contained="
                    << lazyBernsteinResult
                           .final_contained
                    << " final_violation_m="
                    << lazyBernsteinResult
                           .final_max_violation_m
                    << " iterations="
                    << lazyBernsteinResult.iterations
                    << " cuts="
                    << lazyBernsteinResult
                           .activated_cut_count
                    << " active_constraints="
                    << lazyBernsteinResult
                           .active_constraint_count
                    << " duplicate_cuts="
                    << lazyBernsteinResult
                           .duplicate_cut_count
                    << " duplicate_rows="
                    << lazyBernsteinResult
                           .duplicate_row_count
                    << " depth_saturated="
                    << lazyBernsteinResult
                           .depth_saturation_count
                    << " qp_sweeps="
                    << lazyBernsteinResult
                           .total_qp_sweeps
                    << " qp_ms="
                    << lazyBernsteinResult.qp_ms
                    << " cert_ms="
                    << lazyBernsteinResult
                           .certificate_ms
                    << " total_ms="
                    << lazyBernsteinResult.total_ms
                    << " correction_l2_m="
                    << lazyBernsteinResult
                           .correction_l2_m
                    << " max_waypoint_disp_m="
                    << lazyBernsteinResult
                           .max_waypoint_displacement_m);
                
                for (const auto &iter :
                     lazyBernsteinResult
                         .iteration_records)
                {
                    ROS_INFO_STREAM(
                        "TF_LAZY_BERNSTEIN_ITER "
                        << "iter="
                        << iter.iteration
                        << " piece="
                        << iter.piece
                        << " face="
                        << iter.face
                        << " tau="
                        << iter.tau
                        << " pre_violation_m="
                        << iter.pre_violation_m
                        << " depth="
                        << iter.depth
                        << " leaf="
                        << iter.leaf
                        << " interval_begin="
                        << iter.interval_begin
                        << " interval_end="
                        << iter.interval_end
                        << " bound_m="
                        << iter.bernstein_bound_m
                        << " bound_gap_m="
                        << iter.bound_gap_m
                        << " depth_saturated="
                        << iter.depth_saturated
                        << " cut_rows="
                        << iter.cut_rows
                        << " rows_added="
                        << iter.rows_added
                        << " active_rows="
                        << iter.active_rows
                        << " qp_success="
                        << iter.qp_success
                        << " qp_sweeps="
                        << iter.qp_sweeps
                        << " qp_ms="
                        << iter.qp_ms
                        << " post_violation_m="
                        << iter.post_violation_m
                        << " post_worst_piece="
                        << iter.post_worst_piece
                        << " post_worst_face="
                        << iter.post_worst_face
                        << " post_worst_tau="
                        << iter.post_worst_tau
                        << " post_contained="
                        << iter.post_contained);
                }

                hardProjectionResult =
                    traj_relevant::
                        projectMincoToExactSfc(
                            iniState,
                            finState,
                            activeGuideBackendResult
                                .optimized_points,
                            activeGuideBackendResult
                                .optimized_times,
                            activeGuideHPolys,
                            hardProjectedTrajectory,
                            hardProjectedPoints,
                            projectionOptions);
            }

            // ------------------------------------------------------------
            // Cross-check that the hard projection starts from exactly the
            // same soft trajectory certified by runBackendAb().
            // ------------------------------------------------------------
            double hardSourceCertificateMismatchM =
                std::numeric_limits<double>::
                    quiet_NaN();

            if (activeGuideBackendResult
                    .exact_certificate_valid &&
                hardProjectionResult
                    .initial_certificate_valid)
            {
                hardSourceCertificateMismatchM =
                    std::abs(
                        activeGuideBackendResult
                            .exact_max_violation_m -
                        hardProjectionResult
                            .initial_max_violation_m);
            }

            const double proposedHardAfterRouteMs =
                routeMincoGuideBuildMs +
                guideMetricMs +
                activeGuideMs +
                activeGuideBackendResult.setup_ms +
                activeGuideBackendResult.optimize_ms +
                hardProjectionResult.total_ms;

            const double proposedPipelineWallMs =
                std::chrono::duration<
                    double,
                    std::milli>(
                        std::chrono::steady_clock::now() -
                        proposedPipelineWallStarted)
                    .count();

            const double proposedTimingGapMs =
                proposedPipelineWallMs -
                proposedHardAfterRouteMs;

            if (benchmarkProposedMode)
            {
                const bool proposedFinalSuccess =
                    activeGuideSuccess &&
                    activeGuideBackendResult.setup_success &&
                    activeGuideBackendResult.optimize_success &&
                    activeGuideBackendResult.optimized_state_ready &&
                    hardProjectionResult.success &&
                    hardProjectionResult.final_certificate_valid &&
                    hardProjectionResult.final_contained &&
                    hardProjectedTrajectory.getPieceNum() > 0;

                // ========================================================
                // Proposed-only backend diagnostics.
                // No Batch/FIRI fields are printed here.
                // ========================================================
                ROS_INFO_STREAM(
                        "TF_GUIDE_ACTIVE_BACKEND "
                        << "corridor_success="
                        << activeGuideSuccess

                        << " setup_success="
                        << activeGuideBackendResult.setup_success

                        << " opt_success="
                        << activeGuideBackendResult.optimize_success

                        << " corridors="
                        << activeGuideBackendResult.corridor_count

                        << " faces="
                        << activeGuideBackendResult.total_faces

                        << " traj_pieces="
                        << activeGuideBackendResult.trajectory_pieces

                        << " constrained_pieces="
                        << activeGuideBackendResult.constrained_pieces

                        << " final_cost="
                        << activeGuideBackendResult.final_cost

                        << " duration="
                        << activeGuideBackendResult.trajectory_duration

                        << " setup_ms="
                        << activeGuideBackendResult.setup_ms

                        << " opt_ms="
                        << activeGuideBackendResult.optimize_ms

                        << " penalty_initial="
                        << activeGuideBackendResult.corridor_penalty_initial

                        << " penalty_final="
                        << activeGuideBackendResult.corridor_penalty_final

                        << " violation_initial="
                        << activeGuideBackendResult.max_corridor_violation_initial

                        << " violation_final="
                        << activeGuideBackendResult.max_corridor_violation_final

                        << " slack_initial="
                        << activeGuideBackendResult.corridor_slack_initial

                        << " slack_final="
                        << activeGuideBackendResult.corridor_slack_final

                        << " exact_mapping_valid="
                        << activeGuideBackendResult.exact_mapping_valid

                        << " exact_cert_valid="
                        << activeGuideBackendResult.exact_certificate_valid

                        << " exact_contained="
                        << activeGuideBackendResult.exact_contained

                        << " exact_faces_checked="
                        << activeGuideBackendResult.exact_checked_faces

                        << " exact_max_violation_m="
                        << activeGuideBackendResult.exact_max_violation_m

                        << " exact_min_margin_m="
                        << activeGuideBackendResult.exact_min_margin_m

                        << " exact_worst_piece="
                        << activeGuideBackendResult.exact_worst_piece

                        << " exact_worst_face="
                        << activeGuideBackendResult.exact_worst_face

                        << " exact_worst_tau="
                        << activeGuideBackendResult.exact_worst_tau

                        << " exact_worst_t="
                        << activeGuideBackendResult.exact_worst_t

                        << " exact_cert_ms="
                        << activeGuideBackendResult.exact_certificate_ms);

                ROS_INFO_STREAM(
                    "TF_EXACT_HARD_SFC "
                    // ---- 源轨迹信息（来自 runBackendAb） ----
                    << " source_backend_ready="
                    << hardProjectionSourceReady
                    << " source_opt_ms="
                    << activeGuideBackendResult.optimize_ms
                    << " source_cert_mismatch_m="
                    << hardSourceCertificateMismatchM

                    // ---- 硬投影核心结果 ----
                    << " projection_success="
                    << hardProjectionResult.success
                    << " affine_valid="
                    << hardProjectionResult.affine_map_valid

                    << " initial_cert_valid="
                    << hardProjectionResult.initial_certificate_valid
                    << " initial_contained="
                    << hardProjectionResult.initial_contained
                    << " final_cert_valid="
                    << hardProjectionResult.final_certificate_valid
                    << " final_contained="
                    << hardProjectionResult.final_contained

                    << " initial_violation_m="
                    << hardProjectionResult.initial_max_violation_m
                    << " final_violation_m="
                    << hardProjectionResult.final_max_violation_m

                    // ---- 硬投影优化细节 ----
                    << " exchange_iterations="
                    << hardProjectionResult.exchange_iterations
                    << " active_constraints="
                    << hardProjectionResult.active_constraint_count
                    << " qp_sweeps="
                    << hardProjectionResult.total_qp_sweeps
                    << " duplicate_witnesses="
                    << hardProjectionResult.duplicate_witness_count

                    << " correction_l2_m="
                    << hardProjectionResult.correction_l2_m
                    << " max_waypoint_disp_m="
                    << hardProjectionResult.max_waypoint_displacement_m

                    // ---- 能量与时耗 ----
                    << " initial_energy="
                    << hardProjectionResult.initial_energy
                    << " final_energy="
                    << hardProjectionResult.final_energy

                    << " projection_ms="
                    << hardProjectionResult.total_ms
                    << " qp_ms="
                    << hardProjectionResult.qp_ms
                    << " cert_ms="
                    << hardProjectionResult.certificate_ms);

                ROS_INFO_STREAM(
                    "TF_PROPOSED_HARD_TIMING "
                    << "guide_ms="
                    << routeMincoGuideBuildMs

                    << " csgn_ms="
                    << guideMetricMs

                    << " corridor_ms="
                    << activeGuideMs

                    << " setup_ms="
                    << activeGuideBackendResult.setup_ms

                    << " optimize_ms="
                    << activeGuideBackendResult.optimize_ms

                    << " hard_projection_ms="
                    << hardProjectionResult.total_ms

                    << " after_route_ms="
                    << proposedHardAfterRouteMs

                    << " envelope_wall_ms="
                    << proposedPipelineWallMs

                    << " timing_gap_ms="
                    << proposedTimingGapMs

                    << " exact_feasible="
                    << proposedFinalSuccess);

                gcopter_benchmark::
                    BenchmarkRunRecord
                        benchmarkRun;

                bool benchmarkRunReady =
                    false;

                auto emitBenchmarkRun =
                    [&]()
                    {
                        if (!benchmarkRunReady)
                        {
                            return;
                        }
                    
                        if (!benchmarkRunLogger
                                 .logRun(
                                     benchmarkRun))
                        {
                            ROS_ERROR(
                                "Failed to append "
                                "benchmark_runs_v2.csv.");
                        }
                    
                        ROS_INFO_STREAM(
                            "TF_BENCHMARK_RUN "
                            << "case_id="
                            << benchmarkRun.case_id
                        
                            << " fingerprint="
                            << benchmarkRun
                                   .route_fingerprint
                        
                            << " method="
                            << benchmarkRun.method
                        
                            << " variant="
                            << benchmarkRun.variant
                        
                            << " repeat="
                            << benchmarkRun.repeat_id
                        
                            << " success="
                            << benchmarkRun.final_success
                        
                            << " faces="
                            << benchmarkRun.total_faces
                        
                            << " obs_faces="
                            << benchmarkRun.obstacle_faces
                        
                            << " after_route_ms="
                            << benchmarkRun.after_route_ms
                        
                            << " soft_exact="
                            << benchmarkRun
                                   .soft_exact_contained
                        
                            << " final_exact="
                            << benchmarkRun
                                   .final_exact_contained
                        
                            << " active_time_constraints="
                            << benchmarkRun
                                   .active_time_constraints);
                    };

                if (config.benchmarkEnabled &&
                    config.benchmarkMethod ==
                        "proposed")
                {
                    benchmarkRun.case_id =
                        effectiveCaseId;

                    benchmarkRun.route_fingerprint =
                        routeFingerprint;

                    benchmarkRun.method =
                        config.benchmarkMethod;

                    benchmarkRun.variant =
                        config.benchmarkVariant;

                    benchmarkRun.repeat_id =
                        config.benchmarkRepeatId;

                    benchmarkRun.timestamp_s =
                        ros::Time::now().toSec();

                    // ========================================================
                    // Status
                    // ========================================================
                    benchmarkRun.corridor_success =
                        activeGuideSuccess;

                    benchmarkRun.optimizer_setup_success =
                        activeGuideBackendResult
                            .setup_success;

                    benchmarkRun.optimizer_success =
                        activeGuideBackendResult
                            .optimize_success;

                    benchmarkRun.final_success =
                        proposedFinalSuccess;

                    // ========================================================
                    // Corridor complexity / guarantees
                    // ========================================================
                    benchmarkRun.corridor_count =
                        static_cast<int>(
                            activeGuideHPolys.size());

                    benchmarkRun.total_faces =
                        activeGuideTotalFaces;

                    benchmarkRun.obstacle_faces =
                        activeGuideObstacleFaces;

                    benchmarkRun.domain_faces =
                        activeGuideDomainFaces;

                    benchmarkRun.safety_valid_count =
                        activeGuideSafetyCount;

                    benchmarkRun.safety_total_count =
                        static_cast<int>(
                            activeGuideInfos.size());

                    benchmarkRun.overlap_valid_count =
                        activeGuideAdjacentOverlapValidCount;

                    benchmarkRun.overlap_total_count =
                        activeGuideAdjacentOverlapCount;

                    // ========================================================
                    // Active-Witness workload
                    // ========================================================
                    benchmarkRun.candidate_count =
                        activeGuideCandidates;

                    benchmarkRun.active_witness_rounds =
                        activeGuideRounds;

                    benchmarkRun.witness_distance_tests =
                        activeGuideWitnessTests;

                    benchmarkRun.obstacle_face_tests =
                        activeGuideFaceTests;

                    benchmarkRun.redundancy_removed =
                        activeGuideRedundancyRemoved;

                    // ========================================================
                    // Proposed timing
                    // ========================================================
                    benchmarkRun.guide_ms =
                        routeMincoGuideBuildMs;

                    benchmarkRun.csgn_ms =
                        guideMetricMs;

                    benchmarkRun.corridor_ms =
                        activeGuideMs;

                    benchmarkRun.setup_ms =
                        activeGuideBackendResult
                            .setup_ms;

                    benchmarkRun.optimize_ms =
                        activeGuideBackendResult
                            .optimize_ms;

                    benchmarkRun.hard_projection_ms =
                        hardProjectionResult
                            .total_ms;

                    benchmarkRun.after_route_ms =
                        proposedHardAfterRouteMs;

                    // ========================================================
                    // Backend trajectory
                    // ========================================================
                    benchmarkRun.trajectory_piece_count =
                        activeGuideBackendResult
                            .trajectory_pieces;

                    benchmarkRun.trajectory_duration_s =
                        activeGuideBackendResult
                            .trajectory_duration;

                    benchmarkRun.soft_optimizer_cost = 
                        activeGuideBackendResult
                            .final_cost;

                    // ========================================================
                    // Soft trajectory exact certificate
                    // ========================================================
                    benchmarkRun.soft_exact_certificate_valid =
                        activeGuideBackendResult
                            .exact_certificate_valid;

                    benchmarkRun.soft_exact_contained =
                        activeGuideBackendResult
                            .exact_contained;

                    benchmarkRun.soft_exact_max_violation_m =
                        activeGuideBackendResult
                            .exact_max_violation_m;

                    // ========================================================
                    // Hard safety closure
                    // ========================================================
                    benchmarkRun.hard_projection_triggered =
                        hardProjectionResult
                            .initial_certificate_valid &&
                        !hardProjectionResult
                             .initial_contained;

                    benchmarkRun.exchange_iterations =
                        hardProjectionResult
                            .exchange_iterations;

                    benchmarkRun.active_time_constraints =
                        hardProjectionResult
                            .active_constraint_count;

                    benchmarkRun.qp_sweeps =
                        hardProjectionResult
                            .total_qp_sweeps;

                    benchmarkRun.final_exact_certificate_valid =
                        hardProjectionResult
                            .final_certificate_valid;

                    benchmarkRun.final_exact_contained =
                        hardProjectionResult
                            .final_contained;

                    benchmarkRun.final_exact_max_violation_m =
                        hardProjectionResult
                            .final_max_violation_m;

                    benchmarkRun.correction_l2_m =
                        hardProjectionResult
                            .correction_l2_m;

                    benchmarkRun.max_waypoint_disp_m =
                        hardProjectionResult
                            .max_waypoint_displacement_m;

                    benchmarkRun.energy_before =
                        hardProjectionResult
                            .initial_energy;

                    benchmarkRun.energy_after =
                        hardProjectionResult
                            .final_energy;

                    benchmarkRunReady =
                        true;                           
                }

                // ========================================================
                // Success-only trajectory measurements.
                //
                // These variables are declared outside the success branch
                // because the Controlled-Geometry E1 block below is no
                // longer conditioned on backend / hard-closure success.
                // ========================================================
                gcopter_benchmark::
                    FinalTrajectoryMetrics
                        finalTrajectoryEvaluation;

                gcopter_benchmark::
                    FinalTrajectoryMetrics
                        softTrajectoryEvaluation;

                double softEnergyReferenceDelta =
                    std::numeric_limits<double>::
                        quiet_NaN();

                double softDurationReferenceDelta =
                    std::numeric_limits<double>::
                        quiet_NaN();

                bool softHardComparisonValid =
                    false;


                if (proposedFinalSuccess)
                {
                    traj =
                        hardProjectedTrajectory;

                    // ========================================================
                    // Clean trajectory-quality objective used by the paper.
                    //
                    // This deliberately excludes all soft feasibility penalties:
                    //
                    //     J_kin = E_smooth + rho_T * T.
                    //
                    // For the exact-hard trajectory, times are fixed and
                    // hardProjectionResult.final_energy is the MINCO smoothness
                    // energy after the waypoint projection.
                    // ========================================================
                    const double finalTrajectoryDuration =
                        hardProjectedTrajectory
                            .getTotalDuration();
                                    
                    const double finalSmoothnessEnergy =
                        hardProjectionResult
                            .final_energy;
                                    
                    const double finalTimeWeight =
                        config.weightT;
                                    
                    const double finalTimeCost =
                        finalTimeWeight *
                        finalTrajectoryDuration;
                                    
                    const double finalJkin =
                        finalSmoothnessEnergy +
                        finalTimeCost;
                                    
                    const bool finalJkinValid =
                        std::isfinite(
                            finalTrajectoryDuration) &&
                        finalTrajectoryDuration > 0.0 &&
                        std::isfinite(
                            finalSmoothnessEnergy) &&
                        std::isfinite(
                            finalTimeWeight) &&
                        std::isfinite(
                            finalTimeCost) &&
                        std::isfinite(
                            finalJkin);
                        
                    ROS_INFO_STREAM(
                        "TF_FINAL_TRAJ_METRICS "
                        << "source=exact_hard_projection"
                    
                        << " valid="
                        << finalJkinValid
                    
                        << " pieces="
                        << hardProjectedTrajectory
                               .getPieceNum()
                    
                        << " duration_s="
                        << finalTrajectoryDuration
                    
                        << " smoothness_energy="
                        << finalSmoothnessEnergy
                    
                        << " time_weight="
                        << finalTimeWeight
                    
                        << " time_cost="
                        << finalTimeCost
                    
                        << " j_kin="
                        << finalJkin
                    
                        << " soft_optimizer_cost="
                        << activeGuideBackendResult
                               .final_cost
                    
                        << " soft_minus_j_kin="
                        << (activeGuideBackendResult
                                .final_cost -
                            finalJkin));
                        
                    // ========================================================
                    // Unified final-trajectory evaluator.
                    //
                    // This evaluator is method-independent and will later be
                    // reused unchanged by FIRI / Liu / Proposed.
                    // ========================================================
                    finalTrajectoryEvaluation =
                        gcopter_benchmark::
                            evaluateFinalTrajectoryMetrics(
                                hardProjectedTrajectory,
                                hardProjectionResult
                                    .final_energy,
                                config.weightT,
                                config.vehicleMass,
                                config.gravAcc,
                                config.horizDrag,
                                config.vertDrag,
                                config.parasDrag,
                                config.speedEps,
                                1.0e-3);
                            
                    ROS_INFO_STREAM(
                        "TF_FINAL_TRAJ_EVALUATOR "
                        << "source=exact_hard_projection"
                    
                        << " valid="
                        << finalTrajectoryEvaluation
                               .valid
                    
                        << " pieces="
                        << finalTrajectoryEvaluation
                               .piece_count
                    
                        << " metric_step_s="
                        << finalTrajectoryEvaluation
                               .max_sample_step_s
                    
                        << " flatness_samples="
                        << finalTrajectoryEvaluation
                               .flatness_sample_count
                    
                        << " duration_s="
                        << finalTrajectoryEvaluation
                               .duration_s
                    
                        << " length_m="
                        << finalTrajectoryEvaluation
                               .length_m
                    
                        << " smoothness_energy="
                        << finalTrajectoryEvaluation
                               .smoothness_energy
                    
                        << " time_cost="
                        << finalTrajectoryEvaluation
                               .time_cost
                    
                        << " j_kin="
                        << finalTrajectoryEvaluation
                               .j_kin
                    
                        << " j_kin_delta="
                        << (finalTrajectoryEvaluation
                                .j_kin -
                            finalJkin)
                        
                        << " max_vel_mps="
                        << finalTrajectoryEvaluation
                               .max_velocity_mps
                        
                        << " max_acc_mps2="
                        << finalTrajectoryEvaluation
                               .max_acceleration_mps2
                        
                        << " max_body_rate_radps="
                        << finalTrajectoryEvaluation
                               .max_body_rate_radps
                        
                        << " max_tilt_rad="
                        << finalTrajectoryEvaluation
                               .max_tilt_rad
                        
                        << " min_thrust_N="
                        << finalTrajectoryEvaluation
                               .min_thrust_n
                        
                        << " max_thrust_N="
                        << finalTrajectoryEvaluation
                               .max_thrust_n);

                    // ========================================================
                    // Soft-vs-hard post-projection trajectory comparison.
                    //
                    // Reconstruct the EXACT soft GCOPTER source trajectory from
                    // the optimized internal waypoints and times consumed by
                    // the exact-hard projection.
                    //
                    // No optimizer is run here.
                    // This block is outside the frozen planning timers.
                    // ========================================================
                    Trajectory<5>
                        softBackendTrajectory;

                    double softBackendRebuiltEnergy =
                        std::numeric_limits<double>::
                            quiet_NaN();

                    bool softBackendTrajectoryReady =
                        false;

                    if (activeGuideBackendResult
                            .optimized_state_ready)
                    {
                        const int softPieceCount =
                            static_cast<int>(
                                activeGuideBackendResult
                                    .optimized_times
                                    .size());
                            
                        const bool softStateValid =
                            softPieceCount > 0 &&
                            activeGuideBackendResult
                                .optimized_points.rows() == 3 &&
                            activeGuideBackendResult
                                .optimized_points.cols() ==
                                    softPieceCount - 1 &&
                            activeGuideBackendResult
                                .optimized_points.allFinite() &&
                            activeGuideBackendResult
                                .optimized_times.allFinite() &&
                            (activeGuideBackendResult
                                 .optimized_times.array() >
                             0.0)
                                .all();
                            
                        if (softStateValid)
                        {
                            minco::MINCO_S3NU
                                softMinco;
                        
                            softMinco.setConditions(
                                iniState,
                                finState,
                                softPieceCount);
                            
                            softMinco.setParameters(
                                activeGuideBackendResult
                                    .optimized_points,
                                activeGuideBackendResult
                                    .optimized_times);
                            
                            softMinco.getTrajectory(
                                softBackendTrajectory);
                            
                            softMinco.getEnergy(
                                softBackendRebuiltEnergy);
                            
                            softBackendTrajectoryReady =
                                softBackendTrajectory
                                    .getPieceNum() ==
                                    softPieceCount &&
                                std::isfinite(
                                    softBackendRebuiltEnergy);
                        }
                    }

                    if (softBackendTrajectoryReady)
                    {
                        softTrajectoryEvaluation =
                            gcopter_benchmark::
                                evaluateFinalTrajectoryMetrics(
                                    softBackendTrajectory,
                                    softBackendRebuiltEnergy,
                                    config.weightT,
                                    config.vehicleMass,
                                    config.gravAcc,
                                    config.horizDrag,
                                    config.vertDrag,
                                    config.parasDrag,
                                    config.speedEps,
                                    1.0e-3);
                    }

                    softEnergyReferenceDelta =
                        softBackendRebuiltEnergy -
                        hardProjectionResult
                            .initial_energy;

                    softDurationReferenceDelta =
                        softTrajectoryEvaluation
                            .duration_s -
                        activeGuideBackendResult
                            .trajectory_duration;

                    softHardComparisonValid =
                        softBackendTrajectoryReady &&
                        softTrajectoryEvaluation.valid &&
                        finalTrajectoryEvaluation.valid &&
                        std::isfinite(
                            softEnergyReferenceDelta) &&
                        std::isfinite(
                            softDurationReferenceDelta);

                    ROS_INFO_STREAM(
                        "TF_SOFT_HARD_TRAJ_COMPARE "
                    
                        << "valid="
                        << softHardComparisonValid
                    
                        << " soft_rebuild_ready="
                        << softBackendTrajectoryReady
                    
                        << " soft_energy_reference_delta="
                        << softEnergyReferenceDelta
                    
                        << " soft_duration_reference_delta_s="
                        << softDurationReferenceDelta
                    
                        // ----------------------------------------------------
                        // Duration / length
                        // All deltas below are:
                        //
                        //     hard - soft
                        // ----------------------------------------------------
                        << " soft_duration_s="
                        << softTrajectoryEvaluation
                               .duration_s
                    
                        << " hard_duration_s="
                        << finalTrajectoryEvaluation
                               .duration_s
                    
                        << " delta_duration_s="
                        << (finalTrajectoryEvaluation
                                .duration_s -
                            softTrajectoryEvaluation
                                .duration_s)
                        
                        << " soft_length_m="
                        << softTrajectoryEvaluation
                               .length_m
                        
                        << " hard_length_m="
                        << finalTrajectoryEvaluation
                               .length_m
                        
                        << " delta_length_m="
                        << (finalTrajectoryEvaluation
                                .length_m -
                            softTrajectoryEvaluation
                                .length_m)
                        
                        // ----------------------------------------------------
                        // Clean trajectory quality
                        // ----------------------------------------------------
                        << " soft_energy="
                        << softTrajectoryEvaluation
                               .smoothness_energy
                        
                        << " hard_energy="
                        << finalTrajectoryEvaluation
                               .smoothness_energy
                        
                        << " delta_energy="
                        << (finalTrajectoryEvaluation
                                .smoothness_energy -
                            softTrajectoryEvaluation
                                .smoothness_energy)
                        
                        << " soft_j_kin="
                        << softTrajectoryEvaluation
                               .j_kin
                        
                        << " hard_j_kin="
                        << finalTrajectoryEvaluation
                               .j_kin
                        
                        << " delta_j_kin="
                        << (finalTrajectoryEvaluation
                                .j_kin -
                            softTrajectoryEvaluation
                                .j_kin)
                        
                        // ----------------------------------------------------
                        // Translational extrema
                        // ----------------------------------------------------
                        << " soft_max_vel_mps="
                        << softTrajectoryEvaluation
                               .max_velocity_mps
                        
                        << " hard_max_vel_mps="
                        << finalTrajectoryEvaluation
                               .max_velocity_mps
                        
                        << " delta_max_vel_mps="
                        << (finalTrajectoryEvaluation
                                .max_velocity_mps -
                            softTrajectoryEvaluation
                                .max_velocity_mps)
                        
                        << " soft_max_acc_mps2="
                        << softTrajectoryEvaluation
                               .max_acceleration_mps2
                        
                        << " hard_max_acc_mps2="
                        << finalTrajectoryEvaluation
                               .max_acceleration_mps2
                        
                        << " delta_max_acc_mps2="
                        << (finalTrajectoryEvaluation
                                .max_acceleration_mps2 -
                            softTrajectoryEvaluation
                                .max_acceleration_mps2)
                        
                        // ----------------------------------------------------
                        // Flatness-based dynamics
                        // ----------------------------------------------------
                        << " soft_max_body_rate_radps="
                        << softTrajectoryEvaluation
                               .max_body_rate_radps
                        
                        << " hard_max_body_rate_radps="
                        << finalTrajectoryEvaluation
                               .max_body_rate_radps
                        
                        << " delta_max_body_rate_radps="
                        << (finalTrajectoryEvaluation
                                .max_body_rate_radps -
                            softTrajectoryEvaluation
                                .max_body_rate_radps)
                        
                        << " soft_max_tilt_rad="
                        << softTrajectoryEvaluation
                               .max_tilt_rad
                        
                        << " hard_max_tilt_rad="
                        << finalTrajectoryEvaluation
                               .max_tilt_rad
                        
                        << " delta_max_tilt_rad="
                        << (finalTrajectoryEvaluation
                                .max_tilt_rad -
                            softTrajectoryEvaluation
                                .max_tilt_rad)
                        
                        << " soft_min_thrust_N="
                        << softTrajectoryEvaluation
                               .min_thrust_n
                        
                        << " hard_min_thrust_N="
                        << finalTrajectoryEvaluation
                               .min_thrust_n
                        
                        << " delta_min_thrust_N="
                        << (finalTrajectoryEvaluation
                                .min_thrust_n -
                            softTrajectoryEvaluation
                                .min_thrust_n)
                        
                        << " soft_max_thrust_N="
                        << softTrajectoryEvaluation
                               .max_thrust_n
                        
                        << " hard_max_thrust_N="
                        << finalTrajectoryEvaluation
                               .max_thrust_n
                        
                        << " delta_max_thrust_N="
                        << (finalTrajectoryEvaluation
                                .max_thrust_n -
                            softTrajectoryEvaluation
                                .max_thrust_n)
                        
                        // ----------------------------------------------------
                        // Raw margins to configured soft bounds.
                        //
                        // Positive = inside bound.
                        // Negative = exceeds bound.
                        //
                        // No arbitrary feasibility tolerance is introduced.
                        // ----------------------------------------------------
                        << " soft_vel_margin_mps="
                        << (config.maxVelMag -
                            softTrajectoryEvaluation
                                .max_velocity_mps)
                        
                        << " hard_vel_margin_mps="
                        << (config.maxVelMag -
                            finalTrajectoryEvaluation
                                .max_velocity_mps)
                        
                        << " soft_body_rate_margin_radps="
                        << (config.maxBdrMag -
                            softTrajectoryEvaluation
                                .max_body_rate_radps)
                        
                        << " hard_body_rate_margin_radps="
                        << (config.maxBdrMag -
                            finalTrajectoryEvaluation
                                .max_body_rate_radps)
                        
                        << " soft_tilt_margin_rad="
                        << (config.maxTiltAngle -
                            softTrajectoryEvaluation
                                .max_tilt_rad)
                        
                        << " hard_tilt_margin_rad="
                        << (config.maxTiltAngle -
                            finalTrajectoryEvaluation
                                .max_tilt_rad)
                        
                        << " soft_thrust_low_margin_N="
                        << (softTrajectoryEvaluation
                                .min_thrust_n -
                            config.minThrust)
                        
                        << " hard_thrust_low_margin_N="
                        << (finalTrajectoryEvaluation
                                .min_thrust_n -
                            config.minThrust)
                        
                        << " soft_thrust_high_margin_N="
                        << (config.maxThrust -
                            softTrajectoryEvaluation
                                .max_thrust_n)
                        
                        << " hard_thrust_high_margin_N="
                        << (config.maxThrust -
                            finalTrajectoryEvaluation
                                .max_thrust_n));
                    } // proposedFinalSuccess: trajectory evaluation only

                    // ========================================================
                    // Controlled-geometry corridor mapping.
                    //
                    // For the paper's Controlled Geometry protocol we require:
                    //
                    //     one original route segment
                    //         <->
                    //     one Proposed corridor.
                    //
                    // If the overlap shortcut ever removes a corridor, this
                    // mapping becomes invalid and the geometry record must NOT
                    // silently associate the wrong seed segment.
                    // ========================================================
                    const int controlledSegmentCount =
                        std::max(
                            0,
                            static_cast<int>(
                                route.size()) -
                                1);
                            
                    const bool corridorGeometryMappingValid =
                        activeGuideSuccess &&
                        static_cast<int>(
                            activeGuideHPolys.size()) ==
                            controlledSegmentCount &&
                        activeGuideInfos.size() ==
                            activeGuideHPolys.size();

                    double minSeedRadiusM =
                        std::numeric_limits<double>::
                            infinity();

                    double minAdjacentOverlapRadiusM =
                        std::numeric_limits<double>::
                            infinity();

                    int validSeedMetricCount =
                        0;

                    int validOverlapMetricCount =
                        0;

                    int validDirectionalReserveCount =
                        0;

                    double sumHardSymmetricReserveM =
                        0.0;

                    double sumEasySymmetricReserveM =
                        0.0;

                    double minDirectionalReserveM =
                        std::numeric_limits<double>::
                            infinity();

                    double maxDirectionalEigenvalueDelta =
                        0.0;

                    bool protectedSeedSatisfied =
                        corridorGeometryMappingValid;

                    bool protectedOverlapSatisfied =
                        corridorGeometryMappingValid;

                    const double protectedRadiusM =
                        activeGuideOptions
                            .overlap_radius;

                    const double geometryToleranceM =
                        1.0e-6;

                    if (corridorGeometryMappingValid)
                    {
                        for (int corridorId = 0;
                             corridorId <
                                 static_cast<int>(
                                     activeGuideHPolys.size());
                             ++corridorId)
                        {
                            const Eigen::Vector3d &seedA =
                                route[corridorId];
                        
                            const Eigen::Vector3d &seedB =
                                route[corridorId + 1];
                        
                            const auto seedMetric =
                                gcopter_benchmark::
                                    evaluateSegmentSeedRadius(
                                        activeGuideHPolys[
                                            corridorId],
                                        seedA,
                                        seedB);
                                        
                            if (seedMetric.valid)
                            {
                                ++validSeedMetricCount;
                            
                                minSeedRadiusM =
                                    std::min(
                                        minSeedRadiusM,
                                        seedMetric.radius_m);
                                    
                                if (seedMetric.radius_m <
                                    protectedRadiusM -
                                        geometryToleranceM)
                                {
                                    protectedSeedSatisfied =
                                        false;
                                }
                            }
                            else
                            {
                                protectedSeedSatisfied =
                                    false;
                            }
                        
                            double nextOverlapRadiusM =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            bool nextOverlapValid =
                                false;
                        
                            if (corridorId + 1 <
                                static_cast<int>(
                                    activeGuideHPolys.size()))
                            {
                                const Eigen::Vector3d &junction =
                                    route[corridorId + 1];

                                const auto overlapMetric =
                                    gcopter_benchmark::
                                        evaluateJunctionOverlapRadius(
                                            activeGuideHPolys[
                                                corridorId],
                                            activeGuideHPolys[
                                                corridorId + 1],
                                            junction);
                                            
                                nextOverlapValid =
                                    overlapMetric.valid;
                                            
                                nextOverlapRadiusM =
                                    overlapMetric.radius_m;
                                            
                                if (overlapMetric.valid)
                                {
                                    ++validOverlapMetricCount;
                                
                                    minAdjacentOverlapRadiusM =
                                        std::min(
                                            minAdjacentOverlapRadiusM,
                                            overlapMetric.radius_m);
                                        
                                    if (overlapMetric.radius_m <
                                        protectedRadiusM -
                                            geometryToleranceM)
                                    {
                                        protectedOverlapSatisfied =
                                            false;
                                    }
                                }
                                else
                                {
                                    protectedOverlapSatisfied =
                                        false;
                                }
                            }
                        
                            const auto &corridorInfo =
                                activeGuideInfos[
                                    corridorId];
                                
                            const double utilityMin =
                                corridorInfo
                                    .utility_eigenvalues
                                    .minCoeff();
                                
                            const double utilityMax =
                                corridorInfo
                                    .utility_eigenvalues
                                    .maxCoeff();
                                
                            const double utilityAnisotropy =
                                utilityMin > 0.0
                                    ? utilityMax /
                                          utilityMin
                                    : std::numeric_limits<double>::
                                          quiet_NaN();

                            // ========================================================
                            // Final-polytope CSGN directional deformation reserve.
                            //
                            // IMPORTANT:
                            // construction extra radii are only domain allocations.
                            // The values below are measured from the FINAL
                            // obstacle-clipped H-polytope.
                            // ========================================================
                            bool directionalReserveMetricValid =
                                false;

                            double directionalEigenvalueDelta =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    hardDirectionReserve;

                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    middleDirectionReserve;

                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    easyDirectionReserve;

                            double hardReserveRetention =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            double middleReserveRetention =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            double easyReserveRetention =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            double easyToHardSymmetricRatio =
                                std::numeric_limits<double>::
                                    quiet_NaN();

                            if (corridorId <
                                    static_cast<int>(
                                        guideSegmentMetrics.size()) &&
                                guideSegmentMetrics[
                                    corridorId]
                                    .valid &&
                                corridorInfo.metric_valid &&
                                corridorInfo.anisotropic_domain)
                            {
                                Eigen::Matrix3d utility =
                                    guideSegmentMetrics[
                                        corridorId]
                                        .utility;
                                    
                                utility =
                                    0.5 *
                                    (utility +
                                     utility.transpose());
                                    
                                Eigen::SelfAdjointEigenSolver<
                                    Eigen::Matrix3d>
                                    utilitySolver(
                                        utility);
                                    
                                if (utilitySolver.info() ==
                                        Eigen::Success &&
                                    utilitySolver.eigenvalues()
                                            .minCoeff() >
                                        0.0)
                                {
                                    const Eigen::Vector3d
                                        measuredEigenvalues =
                                            utilitySolver
                                                .eigenvalues();
                                
                                    directionalEigenvalueDelta =
                                        (
                                            measuredEigenvalues -
                                            corridorInfo
                                                .utility_eigenvalues)
                                            .cwiseAbs()
                                            .maxCoeff();
                                        
                                    Eigen::Matrix3d directions =
                                        utilitySolver
                                            .eigenvectors();
                                        
                                    // Eigenvector sign is mathematically arbitrary.
                                    // Canonicalize it so positive/negative reserve logs do
                                    // not randomly swap sign across platforms/runs.
                                    for (int directionId = 0;
                                         directionId < 3;
                                         ++directionId)
                                    {
                                        Eigen::Vector3d direction =
                                            directions.col(
                                                directionId);
                                            
                                        Eigen::Index pivotId =
                                            0;
                                            
                                        direction
                                            .cwiseAbs()
                                            .maxCoeff(
                                                &pivotId);
                                            
                                        if (direction(
                                                pivotId) <
                                            0.0)
                                        {
                                            direction =
                                                -direction;
                                        }
                                    
                                        directions.col(
                                            directionId) =
                                            direction;
                                    }
                                
                                    hardDirectionReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                activeGuideHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(0));
                                                
                                    middleDirectionReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                activeGuideHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(1));
                                                
                                    easyDirectionReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                activeGuideHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(2));
                                                
                                    directionalReserveMetricValid =
                                        hardDirectionReserve.valid &&
                                        middleDirectionReserve.valid &&
                                        easyDirectionReserve.valid &&
                                        std::isfinite(
                                            directionalEigenvalueDelta);
                                        
                                    if (directionalReserveMetricValid)
                                    {
                                        const double hardConstruction =
                                            corridorInfo
                                                .extra_radii(0);
                                    
                                        const double middleConstruction =
                                            corridorInfo
                                                .extra_radii(1);
                                    
                                        const double easyConstruction =
                                            corridorInfo
                                                .extra_radii(2);
                                    
                                        if (hardConstruction > 0.0)
                                        {
                                            hardReserveRetention =
                                                hardDirectionReserve
                                                    .symmetric_m /
                                                hardConstruction;
                                        }
                                    
                                        if (middleConstruction > 0.0)
                                        {
                                            middleReserveRetention =
                                                middleDirectionReserve
                                                    .symmetric_m /
                                                middleConstruction;
                                        }
                                    
                                        if (easyConstruction > 0.0)
                                        {
                                            easyReserveRetention =
                                                easyDirectionReserve
                                                    .symmetric_m /
                                                easyConstruction;
                                        }
                                    
                                        if (hardDirectionReserve
                                                .symmetric_m >
                                            1.0e-12)
                                        {
                                            easyToHardSymmetricRatio =
                                                easyDirectionReserve
                                                    .symmetric_m /
                                                hardDirectionReserve
                                                    .symmetric_m;
                                        }

                                        ++validDirectionalReserveCount;

                                        sumHardSymmetricReserveM +=
                                            hardDirectionReserve
                                                .symmetric_m;

                                        sumEasySymmetricReserveM +=
                                            easyDirectionReserve
                                                .symmetric_m;

                                        minDirectionalReserveM =
                                            std::min(
                                                minDirectionalReserveM,
                                                std::min(
                                                    hardDirectionReserve
                                                        .symmetric_m,
                                                    std::min(
                                                        middleDirectionReserve
                                                            .symmetric_m,
                                                        easyDirectionReserve
                                                            .symmetric_m)));
                                                    
                                        maxDirectionalEigenvalueDelta =
                                            std::max(
                                                maxDirectionalEigenvalueDelta,
                                                directionalEigenvalueDelta);
                                    }
                                }
                            }

                            ROS_INFO_STREAM(
                                "TF_CORRIDOR_DIRECTIONAL_RESERVE "
                            
                                << "corridor_id="
                                << corridorId
                            
                                << " valid="
                                << directionalReserveMetricValid
                            
                                << " eigenvalue_delta="
                                << directionalEigenvalueDelta
                            
                                // ----------------------------------------------------
                                // Hard / low-utility direction
                                // ----------------------------------------------------
                                << " hard_pos_m="
                                << hardDirectionReserve
                                       .positive_m
                            
                                << " hard_neg_m="
                                << hardDirectionReserve
                                       .negative_m
                            
                                << " hard_sym_m="
                                << hardDirectionReserve
                                       .symmetric_m
                            
                                << " hard_span_m="
                                << hardDirectionReserve
                                       .span_m
                            
                                << " hard_construction_m="
                                << corridorInfo
                                       .extra_radii(0)
                            
                                << " hard_retention="
                                << hardReserveRetention
                            
                                // ----------------------------------------------------
                                // Middle direction
                                // ----------------------------------------------------
                                << " mid_pos_m="
                                << middleDirectionReserve
                                       .positive_m
                            
                                << " mid_neg_m="
                                << middleDirectionReserve
                                       .negative_m
                            
                                << " mid_sym_m="
                                << middleDirectionReserve
                                       .symmetric_m
                            
                                << " mid_span_m="
                                << middleDirectionReserve
                                       .span_m
                            
                                << " mid_construction_m="
                                << corridorInfo
                                       .extra_radii(1)
                            
                                << " mid_retention="
                                << middleReserveRetention
                            
                                // ----------------------------------------------------
                                // Easy / high-utility direction
                                // ----------------------------------------------------
                                << " easy_pos_m="
                                << easyDirectionReserve
                                       .positive_m
                            
                                << " easy_neg_m="
                                << easyDirectionReserve
                                       .negative_m
                            
                                << " easy_sym_m="
                                << easyDirectionReserve
                                       .symmetric_m
                            
                                << " easy_span_m="
                                << easyDirectionReserve
                                       .span_m
                            
                                << " easy_construction_m="
                                << corridorInfo
                                       .extra_radii(2)
                            
                                << " easy_retention="
                                << easyReserveRetention
                            
                                << " easy_to_hard_sym_ratio="
                                << easyToHardSymmetricRatio);

                            ROS_INFO_STREAM(
                                "TF_CORRIDOR_GEOMETRY "
                                << "corridor_id="
                                << corridorId
                            
                                << " source_segment_id="
                                << corridorId
                            
                                << " seed_valid="
                                << seedMetric.valid
                            
                                << " seed_radius_m="
                                << seedMetric.radius_m
                            
                                << " protected_radius_m="
                                << protectedRadiusM
                            
                                << " junction_overlap_valid="
                                << nextOverlapValid
                            
                                << " junction_overlap_radius_m="
                                << nextOverlapRadiusM
                            
                                << " total_faces="
                                << corridorInfo
                                       .total_face_count
                            
                                << " domain_faces="
                                << corridorInfo
                                       .domain_face_count
                            
                                << " obstacle_faces="
                                << corridorInfo
                                       .selected_obstacle_face_count
                            
                                << " candidate_count="
                                << corridorInfo
                                       .candidate_count
                            
                                << " active_rounds="
                                << corridorInfo
                                       .active_witness_rounds
                            
                                << " safety_verified="
                                << corridorInfo
                                       .safety_verified
                            
                                << " metric_valid="
                                << corridorInfo
                                       .metric_valid
                            
                                << " anisotropic="
                                << corridorInfo
                                       .anisotropic_domain
                            
                                << " utility_anisotropy="
                                << utilityAnisotropy
                            
                                << " utility_eig0="
                                << corridorInfo
                                       .utility_eigenvalues(0)
                            
                                << " utility_eig1="
                                << corridorInfo
                                       .utility_eigenvalues(1)
                            
                                << " utility_eig2="
                                << corridorInfo
                                       .utility_eigenvalues(2)
                            
                                << " extra_radius0_m="
                                << corridorInfo
                                       .extra_radii(0)
                            
                                << " extra_radius1_m="
                                << corridorInfo
                                       .extra_radii(1)
                            
                                << " extra_radius2_m="
                                << corridorInfo
                                       .extra_radii(2)
                            
                                << " mean_metric_damage="
                                << corridorInfo
                                       .mean_metric_damage
                            
                                << " min_metric_damage="
                                << corridorInfo
                                       .min_metric_damage
                            
                                << " max_metric_damage="
                                << corridorInfo
                                       .max_metric_damage);
                        }
                    }
                                
                    if (validSeedMetricCount == 0)
                    {
                        minSeedRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }

                    if (validOverlapMetricCount == 0)
                    {
                        minAdjacentOverlapRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }

                    ROS_INFO_STREAM(
                        "TF_CORRIDOR_GEOMETRY_SUMMARY "
                        << "mapping_valid="
                        << corridorGeometryMappingValid
                    
                        << " route_segments="
                        << controlledSegmentCount
                    
                        << " corridors="
                        << activeGuideHPolys.size()
                    
                        << " seed_metrics_valid="
                        << validSeedMetricCount
                    
                        << " seed_metrics_total="
                        << controlledSegmentCount
                    
                        << " overlap_metrics_valid="
                        << validOverlapMetricCount
                    
                        << " overlap_metrics_total="
                        << std::max(
                               0,
                               controlledSegmentCount - 1)
                        
                        << " protected_radius_m="
                        << protectedRadiusM
                        
                        << " min_seed_radius_m="
                        << minSeedRadiusM
                        
                        << " min_overlap_radius_m="
                        << minAdjacentOverlapRadiusM
                        
                        << " protected_seed_satisfied="
                        << protectedSeedSatisfied
                        
                        << " protected_overlap_satisfied="
                        << protectedOverlapSatisfied);

                    const double meanHardSymmetricReserveM =
                        validDirectionalReserveCount > 0
                            ? sumHardSymmetricReserveM /
                                  static_cast<double>(
                                      validDirectionalReserveCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanEasySymmetricReserveM =
                        validDirectionalReserveCount > 0
                            ? sumEasySymmetricReserveM /
                                  static_cast<double>(
                                      validDirectionalReserveCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    if (validDirectionalReserveCount == 0)
                    {
                        minDirectionalReserveM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    
                        maxDirectionalEigenvalueDelta =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }

                    ROS_INFO_STREAM(
                        "TF_CORRIDOR_DIRECTIONAL_SUMMARY "
                    
                        << "valid="
                        << validDirectionalReserveCount
                    
                        << " total="
                        << controlledSegmentCount
                    
                        << " mean_hard_sym_m="
                        << meanHardSymmetricReserveM
                    
                        << " mean_easy_sym_m="
                        << meanEasySymmetricReserveM
                    
                        << " mean_easy_to_hard_ratio="
                        << (
                            std::isfinite(
                                meanHardSymmetricReserveM) &&
                            meanHardSymmetricReserveM >
                                1.0e-12
                                ? meanEasySymmetricReserveM /
                                      meanHardSymmetricReserveM
                                : std::numeric_limits<double>::
                                      quiet_NaN())
                            
                        << " min_any_sym_m="
                        << minDirectionalReserveM
                            
                        << " max_eigenvalue_delta="
                        << maxDirectionalEigenvalueDelta);

                    // ========================================================
                    // Volume-kernel regression self-test.
                    //
                    // Axis-aligned box:
                    //
                    //     x in [-1, 1]   -> width 2
                    //     y in [-2, 2]   -> width 4
                    //     z in [-3, 3]   -> width 6
                    //
                    // analytical volume = 48 m^3.
                    //
                    // A positively row-scaled copy must produce the same volume,
                    // because H-plane scaling cannot alter the represented set.
                    // ========================================================
                    Eigen::MatrixX4d volumeTestBox(
                        6,
                        4);
                    
                    volumeTestBox <<
                         1.0,  0.0,  0.0, -1.0,
                        -1.0,  0.0,  0.0, -1.0,
                         0.0,  1.0,  0.0, -2.0,
                         0.0, -1.0,  0.0, -2.0,
                         0.0,  0.0,  1.0, -3.0,
                         0.0,  0.0, -1.0, -3.0;
                    
                    Eigen::MatrixX4d
                        scaledVolumeTestBox =
                            volumeTestBox;
                    
                    const Eigen::Matrix<double, 6, 1>
                        volumeTestScales =
                            (
                                Eigen::Matrix<double, 6, 1>()
                                    <<
                                        2.0,
                                        0.5,
                                        3.0,
                                        4.0,
                                        0.25,
                                        5.0)
                                .finished();
                            
                    for (int rowId = 0;
                         rowId < 6;
                         ++rowId)
                    {
                        scaledVolumeTestBox
                            .row(rowId) *=
                            volumeTestScales(
                                rowId);
                    }
                    
                    const auto volumeBoxMetric =
                        gcopter_benchmark::
                            evaluateHPolytopeVolume(
                                volumeTestBox);
                            
                    const auto scaledVolumeBoxMetric =
                        gcopter_benchmark::
                            evaluateHPolytopeVolume(
                                scaledVolumeTestBox);
                            
                    const double volumeSelfTestDeltaM3 =
                        (
                            volumeBoxMetric.valid &&
                            scaledVolumeBoxMetric.valid)
                            ? std::abs(
                                  volumeBoxMetric.volume_m3 -
                                  scaledVolumeBoxMetric.volume_m3)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                            
                    const bool volumeSelfTestValid =
                        volumeBoxMetric.valid &&
                        scaledVolumeBoxMetric.valid &&
                        std::abs(
                            volumeBoxMetric.volume_m3 -
                            48.0) <=
                            1.0e-9 &&
                        std::abs(
                            scaledVolumeBoxMetric.volume_m3 -
                            48.0) <=
                            1.0e-9 &&
                        volumeSelfTestDeltaM3 <=
                            1.0e-9;
                        
                    ROS_INFO_STREAM(
                        "TF_CORRIDOR_VOLUME_SELFTEST "
                    
                        << "valid="
                        << volumeSelfTestValid
                    
                        << " box_valid="
                        << volumeBoxMetric.valid
                    
                        << " scaled_valid="
                        << scaledVolumeBoxMetric.valid
                    
                        << " box_vertices="
                        << volumeBoxMetric.vertex_count
                    
                        << " box_triangles="
                        << volumeBoxMetric.triangle_count
                    
                        << " box_volume_m3="
                        << volumeBoxMetric.volume_m3
                    
                        << " scaled_volume_m3="
                        << scaledVolumeBoxMetric.volume_m3
                    
                        << " scaling_delta_m3="
                        << volumeSelfTestDeltaM3
                    
                        << " box_vertex_violation_m="
                        << volumeBoxMetric
                               .max_vertex_violation_m
                    
                        << " scaled_vertex_violation_m="
                        << scaledVolumeBoxMetric
                               .max_vertex_violation_m);
                    
                    // ========================================================
                    // Common directional-width kernel self-test.
                    //
                    // volumeTestBox is:
                    //
                    //     x in [-1, 1]
                    //     y in [-2, 2]
                    //     z in [-3, 3]
                    //
                    // At q = 0, analytical widths are:
                    //
                    //     Wx = 2 m
                    //     Wy = 4 m
                    //     Wz = 6 m
                    // ========================================================

                    const auto widthTestX =
                        gcopter_benchmark::
                            evaluatePointDirectionalWidth(
                                volumeTestBox,
                                Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::UnitX());
                            
                    const auto widthTestY =
                        gcopter_benchmark::
                            evaluatePointDirectionalWidth(
                                volumeTestBox,
                                Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::UnitY());
                            
                    const auto widthTestZ =
                        gcopter_benchmark::
                            evaluatePointDirectionalWidth(
                                volumeTestBox,
                                Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::UnitZ());
                            
                    const bool widthSelfTestValid =
                        widthTestX.valid &&
                        widthTestY.valid &&
                        widthTestZ.valid &&
                            
                        widthTestX.reference_inside &&
                        widthTestY.reference_inside &&
                        widthTestZ.reference_inside &&
                            
                        std::abs(
                            widthTestX.positive_m -
                            1.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestX.negative_m -
                            1.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestX.width_m -
                            2.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestY.positive_m -
                            2.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestY.negative_m -
                            2.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestY.width_m -
                            4.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestZ.positive_m -
                            3.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestZ.negative_m -
                            3.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            widthTestZ.width_m -
                            6.0) <=
                            1.0e-9;
                        
                    ROS_INFO_STREAM(
                        "TF_DIRECTIONAL_WIDTH_SELFTEST "
                    
                        << "valid="
                        << widthSelfTestValid
                    
                        << " x_pos_m="
                        << widthTestX.positive_m
                    
                        << " x_neg_m="
                        << widthTestX.negative_m
                    
                        << " x_width_m="
                        << widthTestX.width_m
                    
                        << " y_pos_m="
                        << widthTestY.positive_m
                    
                        << " y_neg_m="
                        << widthTestY.negative_m
                    
                        << " y_width_m="
                        << widthTestY.width_m
                    
                        << " z_pos_m="
                        << widthTestZ.positive_m
                    
                        << " z_neg_m="
                        << widthTestZ.negative_m
                    
                        << " z_width_m="
                        << widthTestZ.width_m);

                    // ========================================================
                    // D1c-0 common effective-face kernel self-test.
                    //
                    // Start from the analytical 6-face box:
                    //
                    //     x in [-1, 1]
                    //     y in [-2, 2]
                    //     z in [-3, 3]
                    //
                    // Then add:
                    //
                    //   row 6:
                    //       a positively scaled duplicate of x <= 1
                    //
                    //   row 7:
                    //       redundant loose plane x <= 2
                    //
                    // Expected:
                    //
                    //     raw rows             = 8
                    //     unique plane groups  = 7
                    //     duplicate rows       = 1
                    //     effective faces      = 6
                    //     redundant groups     = 1
                    //
                    // Five of the six true box facets become unbounded support
                    // problems when individually removed; the x <= 1 facet is
                    // still temporarily bounded by redundant x <= 2, so its
                    // support violation is finite (+1 m).
                    //
                    // A positively row-scaled copy must yield identical counts.
                    // ========================================================

                    Eigen::MatrixX4d
                        effectiveFaceTestPoly(
                            8,
                            4);
                        
                        
                    effectiveFaceTestPoly
                        .topRows(6) =
                            volumeTestBox;
                        
                        
                    // Duplicate of x <= 1, scaled by +7.
                    effectiveFaceTestPoly
                        .row(6) =
                            7.0 *
                            volumeTestBox
                                .row(0);
                        
                        
                    // Loose redundant plane:
                    //     x <= 2.
                    effectiveFaceTestPoly
                        .row(7) <<
                            1.0,
                            0.0,
                            0.0,
                            -2.0;
                        
                        
                    Eigen::MatrixX4d
                        scaledEffectiveFaceTestPoly =
                            effectiveFaceTestPoly;
                        
                        
                    const Eigen::Matrix<double, 8, 1>
                        effectiveFaceRowScales =
                            (
                                Eigen::Matrix<double, 8, 1>()
                                    <<
                                        2.0,
                                        0.5,
                                        3.0,
                                        4.0,
                                        0.25,
                                        5.0,
                                        1.7,
                                        0.8)
                                .finished();
                            
                            
                    for (int rowId = 0;
                         rowId < 8;
                         ++rowId)
                    {
                        scaledEffectiveFaceTestPoly
                            .row(rowId) *=
                            effectiveFaceRowScales(
                                rowId);
                    }


                    const auto effectiveFaceTestMetric =
                        gcopter_benchmark::
                            evaluateEffectiveFaces(
                                effectiveFaceTestPoly);
                            
                            
                    const auto scaledEffectiveFaceTestMetric =
                        gcopter_benchmark::
                            evaluateEffectiveFaces(
                                scaledEffectiveFaceTestPoly);
                            
                            
                    const bool effectiveFaceSelfTestValid =
                        effectiveFaceTestMetric.valid &&
                        scaledEffectiveFaceTestMetric.valid &&
                            
                        effectiveFaceTestMetric.raw_rows ==
                            8 &&
                            
                        effectiveFaceTestMetric
                                .unique_plane_groups ==
                            7 &&
                            
                        effectiveFaceTestMetric
                                .duplicate_rows ==
                            1 &&
                            
                        effectiveFaceTestMetric
                                .effective_faces ==
                            6 &&
                            
                        effectiveFaceTestMetric
                                .redundant_plane_groups ==
                            1 &&
                            
                        effectiveFaceTestMetric
                                .unbounded_support_tests ==
                            5 &&
                            
                        scaledEffectiveFaceTestMetric
                                .raw_rows ==
                            effectiveFaceTestMetric
                                .raw_rows &&
                            
                        scaledEffectiveFaceTestMetric
                                .unique_plane_groups ==
                            effectiveFaceTestMetric
                                .unique_plane_groups &&
                            
                        scaledEffectiveFaceTestMetric
                                .duplicate_rows ==
                            effectiveFaceTestMetric
                                .duplicate_rows &&
                            
                        scaledEffectiveFaceTestMetric
                                .effective_faces ==
                            effectiveFaceTestMetric
                                .effective_faces &&
                            
                        scaledEffectiveFaceTestMetric
                                .redundant_plane_groups ==
                            effectiveFaceTestMetric
                                .redundant_plane_groups &&
                            
                        scaledEffectiveFaceTestMetric
                                .unbounded_support_tests ==
                            effectiveFaceTestMetric
                                .unbounded_support_tests;
                            
                            
                    ROS_INFO_STREAM(
                        "TF_EFFECTIVE_FACE_SELFTEST "
                    
                        << "valid="
                        << effectiveFaceSelfTestValid
                    
                        << " base_valid="
                        << effectiveFaceTestMetric.valid
                    
                        << " scaled_valid="
                        << scaledEffectiveFaceTestMetric.valid
                    
                        << " raw_rows="
                        << effectiveFaceTestMetric.raw_rows
                    
                        << " unique_groups="
                        << effectiveFaceTestMetric
                               .unique_plane_groups
                    
                        << " duplicate_rows="
                        << effectiveFaceTestMetric
                               .duplicate_rows
                    
                        << " effective_faces="
                        << effectiveFaceTestMetric
                               .effective_faces
                    
                        << " redundant_groups="
                        << effectiveFaceTestMetric
                               .redundant_plane_groups
                    
                        << " unbounded_tests="
                        << effectiveFaceTestMetric
                               .unbounded_support_tests
                    
                        << " max_redundant_violation_m="
                        << effectiveFaceTestMetric
                               .max_redundant_violation_m
                    
                        << " min_finite_effective_violation_m="
                        << effectiveFaceTestMetric
                               .min_finite_effective_violation_m
                    
                        << " scaled_effective_faces="
                        << scaledEffectiveFaceTestMetric
                               .effective_faces);

                    // ========================================================
                    // D1d-0 common corridor-safety kernel self-test.
                    //
                    // Reuse the analytical box:
                    //
                    //     x in [-1, 1]
                    //     y in [-2, 2]
                    //     z in [-3, 3]
                    //
                    // Test A:
                    //     all obstacle samples are outside or exactly on the
                    //     boundary, and map bounds contain the whole box.
                    //
                    // Test B:
                    //     add q=(0,0,0), which lies one metre inside the
                    //     closest x face.
                    //
                    // Test C:
                    //     no obstacle penetration, but shrink the map lower-x
                    //     bound to -0.5 m.  The box reaches x=-1, hence map
                    //     violation must be +0.5 m.
                    // ========================================================

                    std::vector<Eigen::Vector3d>
                        safetyTestSurfaceSafe;

                    safetyTestSurfaceSafe.emplace_back(
                        1.0,
                        0.0,
                        0.0);   // exact boundary contact
                    
                    safetyTestSurfaceSafe.emplace_back(
                        2.0,
                        0.0,
                        0.0);
                    
                    safetyTestSurfaceSafe.emplace_back(
                        0.0,
                        3.0,
                        0.0);
                    
                    safetyTestSurfaceSafe.emplace_back(
                        0.0,
                        0.0,
                        4.0);
                    
                    
                    const Eigen::Vector3d safetyTestMapLow(
                        -5.0,
                        -5.0,
                        -5.0);
                    
                    const Eigen::Vector3d safetyTestMapHigh(
                        5.0,
                        5.0,
                        5.0);
                    
                    
                    const auto safetyTestSafe =
                        gcopter_benchmark::
                            evaluateCommonCorridorSafety(
                                volumeTestBox,
                                safetyTestSurfaceSafe,
                                safetyTestMapLow,
                                safetyTestMapHigh);
                            
                            
                    std::vector<Eigen::Vector3d>
                        safetyTestSurfaceUnsafe =
                            safetyTestSurfaceSafe;
                            
                    safetyTestSurfaceUnsafe.emplace_back(
                        0.0,
                        0.0,
                        0.0);
                    
                    
                    const auto safetyTestObstacleFailure =
                        gcopter_benchmark::
                            evaluateCommonCorridorSafety(
                                volumeTestBox,
                                safetyTestSurfaceUnsafe,
                                safetyTestMapLow,
                                safetyTestMapHigh);
                            
                            
                    const Eigen::Vector3d
                        safetyTestNarrowMapLow(
                            -0.5,
                            -5.0,
                            -5.0);
                        
                        
                    const auto safetyTestMapFailure =
                        gcopter_benchmark::
                            evaluateCommonCorridorSafety(
                                volumeTestBox,
                                safetyTestSurfaceSafe,
                                safetyTestNarrowMapLow,
                                safetyTestMapHigh);
                            
                            
                    const bool corridorSafetySelfTestValid =
                        // ----------------------------------------------------
                        // Safe case.
                        // Boundary contact is explicitly accepted.
                        // ----------------------------------------------------
                        safetyTestSafe.valid &&
                        safetyTestSafe.safe &&
                        safetyTestSafe
                            .obstacle_surface_safe &&
                        safetyTestSafe
                            .map_contained &&
                            
                        std::abs(
                            safetyTestSafe
                                .min_obstacle_exclusion_margin_m) <=
                            1.0e-9 &&
                        
                        std::abs(
                            safetyTestSafe
                                .max_obstacle_penetration_m) <=
                            1.0e-9 &&
                        
                        
                        // ----------------------------------------------------
                        // Obstacle penetration case.
                        //
                        // At q=(0,0,0):
                        //
                        //     max H residual = -1 m.
                        // ----------------------------------------------------
                        safetyTestObstacleFailure.valid &&
                        !safetyTestObstacleFailure.safe &&
                        !safetyTestObstacleFailure
                             .obstacle_surface_safe &&
                        safetyTestObstacleFailure
                            .map_contained &&
                        
                        std::abs(
                            safetyTestObstacleFailure
                                .min_obstacle_exclusion_margin_m +
                            1.0) <=
                            1.0e-9 &&
                        
                        std::abs(
                            safetyTestObstacleFailure
                                .max_obstacle_penetration_m -
                            1.0) <=
                            1.0e-9 &&
                        
                        
                        // ----------------------------------------------------
                        // Map-containment failure case.
                        //
                        // box min x = -1
                        // map min x = -0.5
                        // violation  = 0.5 m.
                        // ----------------------------------------------------
                        safetyTestMapFailure.valid &&
                        !safetyTestMapFailure.safe &&
                        safetyTestMapFailure
                            .obstacle_surface_safe &&
                        !safetyTestMapFailure
                             .map_contained &&
                        
                        std::abs(
                            safetyTestMapFailure
                                .max_map_violation_m -
                            0.5) <=
                            1.0e-9;
                        
                        
                    ROS_INFO_STREAM(
                        "TF_CORRIDOR_SAFETY_SELFTEST "
                    
                        << "valid="
                        << corridorSafetySelfTestValid
                    
                        << " safe_valid="
                        << safetyTestSafe.valid
                    
                        << " safe="
                        << safetyTestSafe.safe
                    
                        << " safe_obstacle="
                        << safetyTestSafe
                               .obstacle_surface_safe
                    
                        << " safe_map="
                        << safetyTestSafe
                               .map_contained
                    
                        << " safe_min_exclusion_m="
                        << safetyTestSafe
                               .min_obstacle_exclusion_margin_m
                    
                        << " safe_penetration_m="
                        << safetyTestSafe
                               .max_obstacle_penetration_m
                    
                        << " obstacle_failure_safe="
                        << safetyTestObstacleFailure.safe
                    
                        << " obstacle_failure_min_exclusion_m="
                        << safetyTestObstacleFailure
                               .min_obstacle_exclusion_margin_m
                    
                        << " obstacle_failure_penetration_m="
                        << safetyTestObstacleFailure
                               .max_obstacle_penetration_m
                    
                        << " map_failure_safe="
                        << safetyTestMapFailure.safe
                    
                        << " map_failure_contained="
                        << safetyTestMapFailure
                               .map_contained
                    
                        << " map_failure_violation_m="
                        << safetyTestMapFailure
                               .max_map_violation_m);

                    // ========================================================
                    // C2d: Controlled-Geometry CSGN-vs-Identity ablation.
                    //
                    // IMPORTANT:
                    //
                    //   - measurement / ablation only;
                    //   - outside all frozen planning timers;
                    //   - exactly one corridor per original route segment;
                    //   - same ACTIVE_WITNESS constructor;
                    //   - no overlap shortcut;
                    //   - Identity differs only by using the deterministic
                    //     metric-disabled S = I fallback.
                    // ========================================================
                               
                    std::vector<Eigen::MatrixX4d>
                        controlledCsgnHPolys;

                    std::vector<Eigen::MatrixX4d>
                        controlledIdentityHPolys;

                    sfc_gen::TrajectoryRelevantCompactInfos
                        controlledCsgnInfos;

                    sfc_gen::TrajectoryRelevantCompactInfos
                        controlledIdentityInfos;


                    // --------------------------------------------------------
                    // Direct controlled-geometry builder.
                    //
                    // This deliberately calls the SAME inner constructor used
                    // by trajectoryRelevantCompactCover(), but does not execute
                    // the final non-adjacent overlap shortcut.
                    // --------------------------------------------------------
                    auto buildControlledCompactCover =
                        [&](const bool useCsgnMetric,
                            std::vector<Eigen::MatrixX4d> &hPolys,
                            sfc_gen::TrajectoryRelevantCompactInfos &infos)
                            -> bool
                    {
                        hPolys.clear();
                        infos.clear();
                    
                        if (controlledSegmentCount <= 0 ||
                            static_cast<int>(
                                route.size()) !=
                                controlledSegmentCount + 1)
                        {
                            return false;
                        }
                    
                        hPolys.reserve(
                            controlledSegmentCount);
                        
                        infos.reserve(
                            controlledSegmentCount);
                        
                        for (int segmentId = 0;
                             segmentId <
                                 controlledSegmentCount;
                             ++segmentId)
                        {
                            traj_relevant::
                                CompactCorridorOptions
                                    segmentOptions =
                                        activeGuideOptions;
                        
                            // -----------------------------------------------
                            // Deterministic Identity fallback:
                            //
                            //     S = I
                            //
                            // This also avoids arbitrary eigenvectors of an
                            // explicitly diagonalized triply-degenerate I.
                            // -----------------------------------------------
                            segmentOptions.metric_enabled =
                                false;
                        
                            segmentOptions.deformation_utility =
                                Eigen::Matrix3d::Identity();
                        
                            if (useCsgnMetric)
                            {
                                if (segmentId >=
                                        static_cast<int>(
                                            guideSegmentMetrics.size()) ||
                                    !guideSegmentMetrics[
                                         segmentId]
                                         .valid ||
                                    !guideSegmentMetrics[
                                         segmentId]
                                         .utility
                                         .allFinite())
                                {
                                    return false;
                                }
                            
                                segmentOptions.metric_enabled =
                                    true;
                            
                                segmentOptions.deformation_utility =
                                    guideSegmentMetrics[
                                        segmentId]
                                        .utility;
                            }
                        
                            Eigen::MatrixX4d hPoly;
                        
                            traj_relevant::
                                CompactCorridorDiagnostics
                                    diagnostics;
                        
                            const bool generated =
                                traj_relevant::
                                    buildCompactSegmentPolytope(
                                        pc,
                                        voxelMap.getOrigin(),
                                        voxelMap.getCorner(),
                                        route[segmentId],
                                        route[segmentId + 1],
                                        hPoly,
                                        segmentOptions,
                                        &diagnostics);
                                    
                            if (!generated)
                            {
                                infos.push_back(
                                    diagnostics);
                                
                                return false;
                            }
                        
                            hPolys.push_back(
                                hPoly);
                            
                            infos.push_back(
                                diagnostics);
                        }
                    
                        return
                            static_cast<int>(
                                hPolys.size()) ==
                                controlledSegmentCount &&
                            static_cast<int>(
                                infos.size()) ==
                                controlledSegmentCount;
                    };


                    const bool controlledCsgnSuccess =
                        guideSegmentMetricsReady &&
                        buildControlledCompactCover(
                            true,
                            controlledCsgnHPolys,
                            controlledCsgnInfos);
                        
                    const bool controlledIdentitySuccess =
                        guideSegmentMetricsReady &&
                        buildControlledCompactCover(
                            false,
                            controlledIdentityHPolys,
                            controlledIdentityInfos);
                        
                    const bool controlledPairMappingValid =
                        controlledCsgnSuccess &&
                        controlledIdentitySuccess &&
                        static_cast<int>(
                            controlledCsgnHPolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledIdentityHPolys.size()) ==
                            controlledSegmentCount &&
                        controlledCsgnInfos.size() ==
                            controlledCsgnHPolys.size() &&
                        controlledIdentityInfos.size() ==
                            controlledIdentityHPolys.size();
                        
                        
                    // ========================================================
                    // Provenance check:
                    //
                    // On the current regression route the native Proposed path
                    // has no shortcut removal, so the re-built Controlled-CSGN
                    // corridor should be identical face-for-face.
                    // ========================================================
                    bool controlledCsgnNativeMatchValid =
                        controlledCsgnSuccess &&
                        corridorGeometryMappingValid &&
                        controlledCsgnHPolys.size() ==
                            activeGuideHPolys.size();
                        
                    double controlledCsgnNativeMaxPlaneDelta =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                    if (controlledCsgnNativeMatchValid)
                    {
                        controlledCsgnNativeMaxPlaneDelta =
                            0.0;
                    
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            if (controlledCsgnHPolys[
                                    corridorId]
                                    .rows() !=
                                activeGuideHPolys[
                                    corridorId]
                                    .rows())
                            {
                                controlledCsgnNativeMatchValid =
                                    false;
                            
                                break;
                            }
                        
                            controlledCsgnNativeMaxPlaneDelta =
                                std::max(
                                    controlledCsgnNativeMaxPlaneDelta,
                                    (
                                        controlledCsgnHPolys[
                                            corridorId] -
                                        activeGuideHPolys[
                                            corridorId])
                                        .cwiseAbs()
                                        .maxCoeff());
                        }
                    }

                    if (!controlledCsgnNativeMatchValid)
                    {
                        controlledCsgnNativeMaxPlaneDelta =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    // ========================================================
                    // Aggregate geometry / workload.
                    // ========================================================
                    int controlledCsgnTotalFaces =
                        0;

                    int controlledIdentityTotalFaces =
                        0;

                    int controlledCsgnObstacleFaces =
                        0;

                    int controlledIdentityObstacleFaces =
                        0;

                    std::int64_t controlledCsgnCandidates =
                        0;

                    std::int64_t controlledIdentityCandidates =
                        0;

                    std::int64_t controlledCsgnWitnessTests =
                        0;

                    std::int64_t controlledIdentityWitnessTests =
                        0;

                    std::int64_t controlledCsgnFaceTests =
                        0;

                    std::int64_t controlledIdentityFaceTests =
                        0;

                    int controlledCsgnSafetyCount =
                        0;

                    int controlledIdentitySafetyCount =
                        0;

                    int identitySeedValidCount =
                        0;

                    int identityOverlapValidCount =
                        0;

                    double identityMinSeedRadiusM =
                        std::numeric_limits<double>::
                            infinity();

                    double identityMinOverlapRadiusM =
                        std::numeric_limits<double>::
                            infinity();


                    if (controlledPairMappingValid)
                    {
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            const auto &csgnInfo =
                                controlledCsgnInfos[
                                    corridorId];
                                
                            const auto &identityInfo =
                                controlledIdentityInfos[
                                    corridorId];
                                
                            controlledCsgnTotalFaces +=
                                csgnInfo.total_face_count;
                                
                            controlledIdentityTotalFaces +=
                                identityInfo.total_face_count;
                                
                            controlledCsgnObstacleFaces +=
                                csgnInfo
                                    .selected_obstacle_face_count;
                                
                            controlledIdentityObstacleFaces +=
                                identityInfo
                                    .selected_obstacle_face_count;
                                
                            controlledCsgnCandidates +=
                                csgnInfo.candidate_count;
                                
                            controlledIdentityCandidates +=
                                identityInfo.candidate_count;
                                
                            controlledCsgnWitnessTests +=
                                csgnInfo.witness_distance_tests;
                                
                            controlledIdentityWitnessTests +=
                                identityInfo.witness_distance_tests;
                                
                            controlledCsgnFaceTests +=
                                csgnInfo.obstacle_face_tests;
                                
                            controlledIdentityFaceTests +=
                                identityInfo.obstacle_face_tests;
                                
                            controlledCsgnSafetyCount +=
                                csgnInfo.safety_verified
                                    ? 1
                                    : 0;
                                
                            controlledIdentitySafetyCount +=
                                identityInfo.safety_verified
                                    ? 1
                                    : 0;
                                
                                
                            const auto identitySeedMetric =
                                gcopter_benchmark::
                                    evaluateSegmentSeedRadius(
                                        controlledIdentityHPolys[
                                            corridorId],
                                        route[corridorId],
                                        route[corridorId + 1]);
                                        
                            if (identitySeedMetric.valid)
                            {
                                ++identitySeedValidCount;
                            
                                identityMinSeedRadiusM =
                                    std::min(
                                        identityMinSeedRadiusM,
                                        identitySeedMetric
                                            .radius_m);
                            }
                        
                            if (corridorId + 1 <
                                controlledSegmentCount)
                            {
                                const auto identityOverlapMetric =
                                    gcopter_benchmark::
                                        evaluateJunctionOverlapRadius(
                                            controlledIdentityHPolys[
                                                corridorId],
                                            controlledIdentityHPolys[
                                                corridorId + 1],
                                            route[corridorId + 1]);
                                            
                                if (identityOverlapMetric.valid)
                                {
                                    ++identityOverlapValidCount;
                                
                                    identityMinOverlapRadiusM =
                                        std::min(
                                            identityMinOverlapRadiusM,
                                            identityOverlapMetric
                                                .radius_m);
                                }
                            }
                        }
                    }

                    if (identitySeedValidCount == 0)
                    {
                        identityMinSeedRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }

                    if (identityOverlapValidCount == 0)
                    {
                        identityMinOverlapRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    // ========================================================
                    // Paired directional measurement.
                    //
                    // CRITICAL:
                    // Both corridors are measured along the SAME CSGN
                    // reference eigendirections.
                    //
                    // Identity's own eigendirections are deliberately NOT used.
                    // ========================================================
                    std::vector<
                        gcopter_benchmark::
                            BenchmarkCorridorRecord>
                        benchmarkControlledCorridorRecords;

                    benchmarkControlledCorridorRecords.reserve(
                        2 *
                        std::max(
                            0,
                            controlledSegmentCount));
                    
                    int pairedDirectionalValidCount =
                        0;

                    int pairedVolumeValidCount =
                        0;
                                            
                    double sumControlledCsgnVolumeM3 =
                        0.0;
                                            
                    double sumControlledIdentityVolumeM3 =
                        0.0;
                                            
                    double maxVolumeVertexViolationM =
                        -std::numeric_limits<double>::
                            infinity();

                    double sumCsgnHardSymM =
                        0.0;

                    double sumIdentityHardSymM =
                        0.0;

                    double sumCsgnEasySymM =
                        0.0;

                    double sumIdentityEasySymM =
                        0.0;

                    double sumHardPreservation =
                        0.0;

                    double sumEasyPreservation =
                        0.0;

                    double sumPreferentialPreservation =
                        0.0;

                    int preservationValidCount =
                        0;

                    // ========================================================
                    // COMMON cross-method point-directional width.
                    //
                    // Unlike the protected-capsule reserve above, this uses
                    // the original segment midpoint and is directly comparable
                    // with Controlled FIRI.
                    // ========================================================
                    int pairedPointWidthValidCount =
                        0;

                    double sumCsgnHardWidthM =
                        0.0;

                    double sumIdentityHardWidthM =
                        0.0;

                    double sumCsgnMiddleWidthM =
                        0.0;

                    double sumIdentityMiddleWidthM =
                        0.0;

                    double sumCsgnEasyWidthM =
                        0.0;

                    double sumIdentityEasyWidthM =
                        0.0;

                    double minCsgnPointReferenceMarginM =
                        std::numeric_limits<double>::
                            infinity();

                    double minIdentityPointReferenceMarginM =
                        std::numeric_limits<double>::
                            infinity();

                    if (controlledPairMappingValid)
                    {
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            bool pairDirectionalValid =
                                false;
                        
                            bool referenceMetricValid =
                                false;

                            Eigen::Vector3d referenceEigenvalues =
                                Eigen::Vector3d::Constant(
                                    std::numeric_limits<double>::
                                        quiet_NaN());

                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    csgnHardReserve;
                        
                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    csgnMiddleReserve;
                        
                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    csgnEasyReserve;
                        
                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    identityHardReserve;
                        
                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    identityMiddleReserve;
                        
                            gcopter_benchmark::
                                CorridorDirectionalReserveMetric
                                    identityEasyReserve;
                        
                            bool pairPointWidthValid =
                                false;

                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    csgnHardWidth;

                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    csgnMiddleWidth;

                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    csgnEasyWidth;

                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    identityHardWidth;

                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    identityMiddleWidth;

                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    identityEasyWidth;

                            double hardPreservation =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double middlePreservation =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double easyPreservation =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                            double preferentialPreservation =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        
                        
                            if (corridorId <
                                    static_cast<int>(
                                        guideSegmentMetrics.size()) &&
                                guideSegmentMetrics[
                                    corridorId]
                                    .valid)
                            {
                                Eigen::Matrix3d utility =
                                    guideSegmentMetrics[
                                        corridorId]
                                        .utility;
                                    
                                utility =
                                    0.5 *
                                    (utility +
                                     utility.transpose());
                                    
                                Eigen::SelfAdjointEigenSolver<
                                    Eigen::Matrix3d>
                                    utilitySolver(
                                        utility);
                                    
                                if (utilitySolver.info() ==
                                        Eigen::Success &&
                                    utilitySolver.eigenvalues()
                                            .minCoeff() >
                                        0.0)
                                {
                                    referenceMetricValid =
                                        true;
                                
                                    referenceEigenvalues =
                                        utilitySolver
                                            .eigenvalues();
                                
                                    Eigen::Matrix3d directions =
                                        utilitySolver
                                            .eigenvectors();
                                
                                    // Deterministic sign convention.
                                    for (int directionId = 0;
                                         directionId < 3;
                                         ++directionId)
                                    {
                                        Eigen::Vector3d direction =
                                            directions.col(
                                                directionId);
                                            
                                        Eigen::Index pivotId =
                                            0;
                                            
                                        direction
                                            .cwiseAbs()
                                            .maxCoeff(
                                                &pivotId);
                                            
                                        if (direction(
                                                pivotId) <
                                            0.0)
                                        {
                                            direction =
                                                -direction;
                                        }
                                    
                                        directions.col(
                                            directionId) =
                                            direction;
                                    }
                                
                                
                                    const Eigen::Vector3d &seedA =
                                        route[corridorId];

                                    const Eigen::Vector3d &seedB =
                                        route[corridorId + 1];

                                    // COMMON reference point for cross-method E1 geometry.
                                    const Eigen::Vector3d
                                        directionalReferencePoint =
                                            0.5 *
                                            (
                                                seedA +
                                                seedB);
                                
                                
                                    csgnHardReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                controlledCsgnHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(0));
                                                
                                    csgnMiddleReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                controlledCsgnHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(1));
                                                
                                    csgnEasyReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                controlledCsgnHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(2));
                                                
                                                
                                    identityHardReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                controlledIdentityHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(0));
                                                
                                    identityMiddleReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                controlledIdentityHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(1));
                                                
                                    identityEasyReserve =
                                        gcopter_benchmark::
                                            evaluateProtectedSegmentDirectionalReserve(
                                                controlledIdentityHPolys[
                                                    corridorId],
                                                seedA,
                                                seedB,
                                                protectedRadiusM,
                                                directions.col(2));
                                                
                                    // ====================================================
                                    // COMMON point-directional width.
                                    //
                                    // Both Controlled CSGN and Controlled Identity are
                                    // measured through exactly the SAME segment midpoint
                                    // and the SAME CSGN reference eigendirections.
                                    //
                                    // These values are therefore directly comparable with
                                    // TF_CONTROLLED_FIRI_GEOMETRY.
                                    // ====================================================

                                    csgnHardWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                controlledCsgnHPolys[
                                                    corridorId],
                                                directionalReferencePoint,
                                                directions.col(0));
                                                
                                    csgnMiddleWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                controlledCsgnHPolys[
                                                    corridorId],
                                                directionalReferencePoint,
                                                directions.col(1));
                                                
                                    csgnEasyWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                controlledCsgnHPolys[
                                                    corridorId],
                                                directionalReferencePoint,
                                                directions.col(2));
                                                
                                                
                                    identityHardWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                controlledIdentityHPolys[
                                                    corridorId],
                                                directionalReferencePoint,
                                                directions.col(0));
                                                
                                    identityMiddleWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                controlledIdentityHPolys[
                                                    corridorId],
                                                directionalReferencePoint,
                                                directions.col(1));
                                                
                                    identityEasyWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                controlledIdentityHPolys[
                                                    corridorId],
                                                directionalReferencePoint,
                                                directions.col(2));
                                                
                                                
                                    pairPointWidthValid =
                                        csgnHardWidth.valid &&
                                        csgnMiddleWidth.valid &&
                                        csgnEasyWidth.valid &&
                                                
                                        identityHardWidth.valid &&
                                        identityMiddleWidth.valid &&
                                        identityEasyWidth.valid &&
                                                
                                        csgnHardWidth.reference_inside &&
                                        csgnMiddleWidth.reference_inside &&
                                        csgnEasyWidth.reference_inside &&
                                                
                                        identityHardWidth.reference_inside &&
                                        identityMiddleWidth.reference_inside &&
                                        identityEasyWidth.reference_inside;
                                                
                                                
                                    if (pairPointWidthValid)
                                    {
                                        ++pairedPointWidthValidCount;
                                    
                                        sumCsgnHardWidthM +=
                                            csgnHardWidth.width_m;
                                    
                                        sumIdentityHardWidthM +=
                                            identityHardWidth.width_m;
                                    
                                        sumCsgnMiddleWidthM +=
                                            csgnMiddleWidth.width_m;
                                    
                                        sumIdentityMiddleWidthM +=
                                            identityMiddleWidth.width_m;
                                    
                                        sumCsgnEasyWidthM +=
                                            csgnEasyWidth.width_m;
                                    
                                        sumIdentityEasyWidthM +=
                                            identityEasyWidth.width_m;
                                    
                                    
                                        const double csgnReferenceMarginM =
                                            std::min(
                                                csgnHardWidth
                                                    .min_reference_margin_m,
                                                std::min(
                                                    csgnMiddleWidth
                                                        .min_reference_margin_m,
                                                    csgnEasyWidth
                                                        .min_reference_margin_m));
                                                
                                        const double identityReferenceMarginM =
                                            std::min(
                                                identityHardWidth
                                                    .min_reference_margin_m,
                                                std::min(
                                                    identityMiddleWidth
                                                        .min_reference_margin_m,
                                                    identityEasyWidth
                                                        .min_reference_margin_m));
                                                
                                                
                                        minCsgnPointReferenceMarginM =
                                            std::min(
                                                minCsgnPointReferenceMarginM,
                                                csgnReferenceMarginM);
                                            
                                        minIdentityPointReferenceMarginM =
                                            std::min(
                                                minIdentityPointReferenceMarginM,
                                                identityReferenceMarginM);
                                    }                                                

                                    pairDirectionalValid =
                                        csgnHardReserve.valid &&
                                        csgnMiddleReserve.valid &&
                                        csgnEasyReserve.valid &&
                                        identityHardReserve.valid &&
                                        identityMiddleReserve.valid &&
                                        identityEasyReserve.valid;
                                                
                                    if (pairDirectionalValid)
                                    {
                                        ++pairedDirectionalValidCount;
                                    
                                        sumCsgnHardSymM +=
                                            csgnHardReserve
                                                .symmetric_m;
                                    
                                        sumIdentityHardSymM +=
                                            identityHardReserve
                                                .symmetric_m;
                                    
                                        sumCsgnEasySymM +=
                                            csgnEasyReserve
                                                .symmetric_m;
                                    
                                        sumIdentityEasySymM +=
                                            identityEasyReserve
                                                .symmetric_m;
                                    
                                    
                                        if (identityHardReserve
                                                .symmetric_m >
                                                1.0e-12 &&
                                            identityMiddleReserve
                                                .symmetric_m >
                                                1.0e-12 &&
                                            identityEasyReserve
                                                .symmetric_m >
                                                1.0e-12)
                                        {
                                            hardPreservation =
                                                csgnHardReserve
                                                    .symmetric_m /
                                                identityHardReserve
                                                    .symmetric_m;
                                        
                                            middlePreservation =
                                                csgnMiddleReserve
                                                    .symmetric_m /
                                                identityMiddleReserve
                                                    .symmetric_m;
                                        
                                            easyPreservation =
                                                csgnEasyReserve
                                                    .symmetric_m /
                                                identityEasyReserve
                                                    .symmetric_m;
                                        
                                            if (hardPreservation >
                                                1.0e-12)
                                            {
                                                preferentialPreservation =
                                                    easyPreservation /
                                                    hardPreservation;
                                            }
                                        
                                            if (std::isfinite(
                                                    hardPreservation) &&
                                                std::isfinite(
                                                    easyPreservation) &&
                                                std::isfinite(
                                                    preferentialPreservation))
                                            {
                                                ++preservationValidCount;
                                            
                                                sumHardPreservation +=
                                                    hardPreservation;
                                            
                                                sumEasyPreservation +=
                                                    easyPreservation;
                                            
                                                sumPreferentialPreservation +=
                                                    preferentialPreservation;
                                            }
                                        }
                                    }
                                }
                            }
                        
                            const auto controlledCsgnSeedMetric =
                                gcopter_benchmark::
                                    evaluateSegmentSeedRadius(
                                        controlledCsgnHPolys[
                                            corridorId],
                                        route[corridorId],
                                        route[corridorId + 1]);
                                        
                            const auto controlledIdentitySeedMetric =
                                gcopter_benchmark::
                                    evaluateSegmentSeedRadius(
                                        controlledIdentityHPolys[
                                            corridorId],
                                        route[corridorId],
                                        route[corridorId + 1]);
                                        
                                        
                            gcopter_benchmark::
                                CorridorOverlapMetric
                                    controlledCsgnOverlapMetric;
                                        
                            gcopter_benchmark::
                                CorridorOverlapMetric
                                    controlledIdentityOverlapMetric;
                                        
                            if (corridorId + 1 <
                                controlledSegmentCount)
                            {
                                controlledCsgnOverlapMetric =
                                    gcopter_benchmark::
                                        evaluateJunctionOverlapRadius(
                                            controlledCsgnHPolys[
                                                corridorId],
                                            controlledCsgnHPolys[
                                                corridorId + 1],
                                            route[corridorId + 1]);
                                            
                                controlledIdentityOverlapMetric =
                                    gcopter_benchmark::
                                        evaluateJunctionOverlapRadius(
                                            controlledIdentityHPolys[
                                                corridorId],
                                            controlledIdentityHPolys[
                                                corridorId + 1],
                                            route[corridorId + 1]);
                            }
                        
                            const auto controlledCsgnVolumeMetric =
                                gcopter_benchmark::
                                    evaluateHPolytopeVolume(
                                        controlledCsgnHPolys[
                                            corridorId]);
                                        
                            const auto controlledIdentityVolumeMetric =
                                gcopter_benchmark::
                                    evaluateHPolytopeVolume(
                                        controlledIdentityHPolys[
                                            corridorId]);
                                        
                            const bool pairedVolumeValid =
                                controlledCsgnVolumeMetric.valid &&
                                controlledIdentityVolumeMetric.valid;

                            ROS_INFO_STREAM(
                                "TF_CORRIDOR_VOLUME "
                            
                                << "corridor_id="
                                << corridorId
                            
                                << " valid="
                                << pairedVolumeValid
                            
                                << " csgn_valid="
                                << controlledCsgnVolumeMetric
                                       .valid
                            
                                << " identity_valid="
                                << controlledIdentityVolumeMetric
                                       .valid
                            
                                << " csgn_vertices="
                                << controlledCsgnVolumeMetric
                                       .vertex_count
                            
                                << " identity_vertices="
                                << controlledIdentityVolumeMetric
                                       .vertex_count
                            
                                << " csgn_triangles="
                                << controlledCsgnVolumeMetric
                                       .triangle_count
                            
                                << " identity_triangles="
                                << controlledIdentityVolumeMetric
                                       .triangle_count
                            
                                << " csgn_volume_m3="
                                << controlledCsgnVolumeMetric
                                       .volume_m3
                            
                                << " identity_volume_m3="
                                << controlledIdentityVolumeMetric
                                       .volume_m3
                            
                                << " csgn_vertex_violation_m="
                                << controlledCsgnVolumeMetric
                                       .max_vertex_violation_m
                            
                                << " identity_vertex_violation_m="
                                << controlledIdentityVolumeMetric
                                       .max_vertex_violation_m);
                            
                                if (pairedVolumeValid)
                                {
                                    ++pairedVolumeValidCount;
                                
                                    sumControlledCsgnVolumeM3 +=
                                        controlledCsgnVolumeMetric
                                            .volume_m3;
                                
                                    sumControlledIdentityVolumeM3 +=
                                        controlledIdentityVolumeMetric
                                            .volume_m3;
                                
                                    maxVolumeVertexViolationM =
                                        std::max(
                                            maxVolumeVertexViolationM,
                                            std::max(
                                                controlledCsgnVolumeMetric
                                                    .max_vertex_violation_m,
                                                controlledIdentityVolumeMetric
                                                    .max_vertex_violation_m));
                                }

                            auto appendControlledCorridorRecord =
                                [&](const std::string &variantName,
                                    const std::string &constructionBasis,
                                    const traj_relevant::
                                        CompactCorridorDiagnostics &info,
                                    const bool constructionUsesCsgnMetric,
                                    const gcopter_benchmark::
                                        CorridorSeedMetric &seedMetric,
                                    const gcopter_benchmark::
                                        CorridorOverlapMetric &overlapMetric,
                                    const gcopter_benchmark::
                                        CorridorDirectionalReserveMetric &hardReserve,
                                    const gcopter_benchmark::
                                        CorridorDirectionalReserveMetric &middleReserve,
                                    const gcopter_benchmark::
                                        CorridorDirectionalReserveMetric &easyReserve)
                            {
                                gcopter_benchmark::
                                    BenchmarkCorridorRecord
                                        record;
                            
                                record.case_id =
                                    effectiveCaseId;
                            
                                record.route_fingerprint =
                                    routeFingerprint;
                            
                                record.method =
                                    "proposed";
                            
                                record.variant =
                                    variantName;
                            
                                record.repeat_id =
                                    config.benchmarkRepeatId;
                            
                                record.timestamp_s =
                                    benchmarkRunReady
                                        ? benchmarkRun.timestamp_s
                                        : ros::Time::now().toSec();
                            
                                record.corridor_id =
                                    corridorId;
                            
                                record.source_segment_id =
                                    corridorId;
                            
                                record.geometry_mapping_valid =
                                    controlledPairMappingValid;
                            
                                record.geometry_protocol =
                                    "controlled_geometry";
                            
                                record.construction_direction_basis =
                                    constructionBasis;
                            
                                record.reference_direction_source =
                                    "direct_minco_csgn";
                            
                            
                                // ====================================================
                                // Seed / neighboring overlap
                                // ====================================================
                                record.seed_metric_valid =
                                    seedMetric.valid;
                            
                                record.seed_radius_m =
                                    seedMetric.radius_m;
                            
                                record.protected_radius_m =
                                    protectedRadiusM;
                            
                                record.junction_overlap_valid =
                                    overlapMetric.valid;
                            
                                record.junction_overlap_radius_m =
                                    overlapMetric.radius_m;
                            
                            
                                // ====================================================
                                // Faces
                                // ====================================================
                                record.total_faces =
                                    info.total_face_count;
                            
                                record.domain_faces =
                                    info.domain_face_count;
                            
                                record.obstacle_faces =
                                    info.selected_obstacle_face_count;
                            
                            
                                // ====================================================
                                // Constraint-generation workload
                                // ====================================================
                                record.input_obstacle_count =
                                    info.input_obstacle_count;
                            
                                record.local_obstacle_count =
                                    info.local_obstacle_count;
                            
                                record.candidate_count =
                                    info.candidate_count;
                            
                                record.generated_candidate_count =
                                    info.generated_candidate_count;
                            
                                record.active_witness_rounds =
                                    info.active_witness_rounds;
                            
                                record.witness_distance_tests =
                                    info.witness_distance_tests;
                            
                                record.obstacle_face_tests =
                                    info.obstacle_face_tests;
                            
                                record.greedy_obstacle_face_count =
                                    info.greedy_obstacle_face_count;
                            
                                record.redundancy_removed =
                                    info.redundancy_removed;
                            
                                record.safety_verified =
                                    info.safety_verified;
                            
                                record.overlap_guaranteed =
                                    info.overlap_guaranteed;
                            
                            
                                // ====================================================
                                // Construction metric.
                                //
                                // For Identity this intentionally describes the
                                // deterministic metric-disabled constructor.
                                // ====================================================
                                record.metric_valid =
                                    info.metric_valid;
                            
                                record.anisotropic_domain =
                                    info.anisotropic_domain;
                            
                                record.utility_eig0 =
                                    info.utility_eigenvalues(0);
                            
                                record.utility_eig1 =
                                    info.utility_eigenvalues(1);
                            
                                record.utility_eig2 =
                                    info.utility_eigenvalues(2);
                            
                                const double constructionUtilityMin =
                                    info.utility_eigenvalues
                                        .minCoeff();
                            
                                const double constructionUtilityMax =
                                    info.utility_eigenvalues
                                        .maxCoeff();
                            
                                record.utility_anisotropy =
                                    constructionUtilityMin > 0.0
                                        ? constructionUtilityMax /
                                              constructionUtilityMin
                                        : std::numeric_limits<double>::
                                              quiet_NaN();
                            
                                record.construction_extra_radius0_m =
                                    info.extra_radii(0);
                            
                                record.construction_extra_radius1_m =
                                    info.extra_radii(1);
                            
                                record.construction_extra_radius2_m =
                                    info.extra_radii(2);
                            
                                record.mean_metric_damage =
                                    info.mean_metric_damage;
                            
                                record.min_metric_damage =
                                    info.min_metric_damage;
                            
                                record.max_metric_damage =
                                    info.max_metric_damage;
                            
                                if (constructionUsesCsgnMetric &&
                                    corridorId <
                                        static_cast<int>(
                                            guideSegmentMetrics.size()))
                                {
                                    record.metric_source_piece_id =
                                        guideSegmentMetrics[
                                            corridorId]
                                            .source_piece_id;
                                        
                                    record.metric_mapping_distance =
                                        guideSegmentMetrics[
                                            corridorId]
                                            .mapping_distance;
                                }
                            
                            
                                // ====================================================
                                // COMMON reference CSGN metric.
                                //
                                // Both CSGN and Identity rows receive exactly the same
                                // reference metric provenance.
                                // ====================================================
                                record.reference_metric_valid =
                                    referenceMetricValid;
                            
                                if (referenceMetricValid)
                                {
                                    record.reference_utility_eig0 =
                                        referenceEigenvalues(0);
                                
                                    record.reference_utility_eig1 =
                                        referenceEigenvalues(1);
                                
                                    record.reference_utility_eig2 =
                                        referenceEigenvalues(2);
                                
                                    record.reference_metric_source_piece_id =
                                        guideSegmentMetrics[
                                            corridorId]
                                            .source_piece_id;
                                        
                                    record.reference_metric_mapping_distance =
                                        guideSegmentMetrics[
                                            corridorId]
                                            .mapping_distance;
                                }
                            
                            
                                // ====================================================
                                // Raw final directional reserve.
                                // ====================================================
                                record.directional_reserve_valid =
                                    pairDirectionalValid;
                            
                                record.hard_positive_m =
                                    hardReserve.positive_m;
                            
                                record.hard_negative_m =
                                    hardReserve.negative_m;
                            
                                record.hard_symmetric_m =
                                    hardReserve.symmetric_m;
                            
                                record.hard_span_m =
                                    hardReserve.span_m;
                            
                                record.middle_positive_m =
                                    middleReserve.positive_m;
                            
                                record.middle_negative_m =
                                    middleReserve.negative_m;
                            
                                record.middle_symmetric_m =
                                    middleReserve.symmetric_m;
                            
                                record.middle_span_m =
                                    middleReserve.span_m;
                            
                                record.easy_positive_m =
                                    easyReserve.positive_m;
                            
                                record.easy_negative_m =
                                    easyReserve.negative_m;
                            
                                record.easy_symmetric_m =
                                    easyReserve.symmetric_m;
                            
                                record.easy_span_m =
                                    easyReserve.span_m;
                            
                                benchmarkControlledCorridorRecords
                                    .push_back(
                                        record);
                            };

                            appendControlledCorridorRecord(
                                "csgn_active_controlled",
                                "csgn_eigenbasis",
                                controlledCsgnInfos[
                                    corridorId],
                                true,
                                controlledCsgnSeedMetric,
                                controlledCsgnOverlapMetric,
                                csgnHardReserve,
                                csgnMiddleReserve,
                                csgnEasyReserve);
                                
                            appendControlledCorridorRecord(
                                "identity_active_controlled",
                                "world_xyz_identity",
                                controlledIdentityInfos[
                                    corridorId],
                                false,
                                controlledIdentitySeedMetric,
                                controlledIdentityOverlapMetric,
                                identityHardReserve,
                                identityMiddleReserve,
                                identityEasyReserve);

                            ROS_INFO_STREAM(
                                "TF_CSGN_IDENTITY_GEOMETRY "
                            
                                << "corridor_id="
                                << corridorId
                            
                                << " valid="
                                << pairDirectionalValid
                            
                                << " csgn_hard_sym_m="
                                << csgnHardReserve
                                       .symmetric_m
                            
                                << " identity_hard_sym_m="
                                << identityHardReserve
                                       .symmetric_m
                            
                                << " hard_preservation="
                                << hardPreservation
                            
                                << " csgn_mid_sym_m="
                                << csgnMiddleReserve
                                       .symmetric_m
                            
                                << " identity_mid_sym_m="
                                << identityMiddleReserve
                                       .symmetric_m
                            
                                << " mid_preservation="
                                << middlePreservation
                            
                                << " csgn_easy_sym_m="
                                << csgnEasyReserve
                                       .symmetric_m
                            
                                << " identity_easy_sym_m="
                                << identityEasyReserve
                                       .symmetric_m
                            
                                << " easy_preservation="
                                << easyPreservation
                            
                                << " preferential_preservation="
                                << preferentialPreservation
                            
                                << " csgn_faces="
                                << controlledCsgnInfos[
                                       corridorId]
                                       .total_face_count
                                
                                << " identity_faces="
                                << controlledIdentityInfos[
                                       corridorId]
                                       .total_face_count
                                
                                << " csgn_obs_faces="
                                << controlledCsgnInfos[
                                       corridorId]
                                       .selected_obstacle_face_count
                                
                                << " identity_obs_faces="
                                << controlledIdentityInfos[
                                       corridorId]
                                       .selected_obstacle_face_count
                                
                                << " csgn_candidates="
                                << controlledCsgnInfos[
                                       corridorId]
                                       .candidate_count
                                
                                << " identity_candidates="
                                << controlledIdentityInfos[
                                       corridorId]
                                       .candidate_count);

                            ROS_INFO_STREAM(
                                "TF_CSGN_IDENTITY_POINT_WIDTH "
                            
                                << "corridor_id="
                                << corridorId
                            
                                << " valid="
                                << pairPointWidthValid
                            
                                << " csgn_reference_margin_m="
                                << csgnHardWidth
                                       .min_reference_margin_m
                            
                                << " identity_reference_margin_m="
                                << identityHardWidth
                                       .min_reference_margin_m
                            
                                << " csgn_hard_width_m="
                                << csgnHardWidth.width_m
                            
                                << " identity_hard_width_m="
                                << identityHardWidth.width_m
                            
                                << " csgn_mid_width_m="
                                << csgnMiddleWidth.width_m
                            
                                << " identity_mid_width_m="
                                << identityMiddleWidth.width_m
                            
                                << " csgn_easy_width_m="
                                << csgnEasyWidth.width_m
                            
                                << " identity_easy_width_m="
                                << identityEasyWidth.width_m);
                        }
                    }


                    // ========================================================
                    // Aggregate paired ablation summary.
                    // ========================================================
                    const double meanCsgnHardSymM =
                        pairedDirectionalValidCount > 0
                            ? sumCsgnHardSymM /
                                  static_cast<double>(
                                      pairedDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanIdentityHardSymM =
                        pairedDirectionalValidCount > 0
                            ? sumIdentityHardSymM /
                                  static_cast<double>(
                                      pairedDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanCsgnEasySymM =
                        pairedDirectionalValidCount > 0
                            ? sumCsgnEasySymM /
                                  static_cast<double>(
                                      pairedDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanIdentityEasySymM =
                        pairedDirectionalValidCount > 0
                            ? sumIdentityEasySymM /
                                  static_cast<double>(
                                      pairedDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanHardPreservation =
                        preservationValidCount > 0
                            ? sumHardPreservation /
                                  static_cast<double>(
                                      preservationValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanEasyPreservation =
                        preservationValidCount > 0
                            ? sumEasyPreservation /
                                  static_cast<double>(
                                      preservationValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanPreferentialPreservation =
                        preservationValidCount > 0
                            ? sumPreferentialPreservation /
                                  static_cast<double>(
                                      preservationValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    ROS_INFO_STREAM(
                        "TF_CSGN_IDENTITY_SUMMARY "
                    
                        << "controlled_csgn_success="
                        << controlledCsgnSuccess
                    
                        << " identity_success="
                        << controlledIdentitySuccess
                    
                        << " mapping_valid="
                        << controlledPairMappingValid
                    
                        << " csgn_native_match_valid="
                        << controlledCsgnNativeMatchValid
                    
                        << " csgn_native_max_plane_delta="
                        << controlledCsgnNativeMaxPlaneDelta
                    
                        << " directional_valid="
                        << pairedDirectionalValidCount
                    
                        << " directional_total="
                        << controlledSegmentCount
                    
                        << " preservation_valid="
                        << preservationValidCount
                    
                        << " csgn_faces="
                        << controlledCsgnTotalFaces
                    
                        << " identity_faces="
                        << controlledIdentityTotalFaces
                    
                        << " csgn_obs_faces="
                        << controlledCsgnObstacleFaces
                    
                        << " identity_obs_faces="
                        << controlledIdentityObstacleFaces
                    
                        << " csgn_candidates="
                        << controlledCsgnCandidates
                    
                        << " identity_candidates="
                        << controlledIdentityCandidates
                    
                        << " csgn_witness_tests="
                        << controlledCsgnWitnessTests
                    
                        << " identity_witness_tests="
                        << controlledIdentityWitnessTests
                    
                        << " csgn_face_tests="
                        << controlledCsgnFaceTests
                    
                        << " identity_face_tests="
                        << controlledIdentityFaceTests
                    
                        << " csgn_safety="
                        << controlledCsgnSafetyCount
                    
                        << " identity_safety="
                        << controlledIdentitySafetyCount
                    
                        << " identity_seed_valid="
                        << identitySeedValidCount
                    
                        << " identity_seed_total="
                        << controlledSegmentCount
                    
                        << " identity_min_seed_m="
                        << identityMinSeedRadiusM
                    
                        << " identity_overlap_valid="
                        << identityOverlapValidCount
                    
                        << " identity_overlap_total="
                        << std::max(
                               0,
                               controlledSegmentCount - 1)
                        
                        << " identity_min_overlap_m="
                        << identityMinOverlapRadiusM
                        
                        << " mean_csgn_hard_sym_m="
                        << meanCsgnHardSymM
                        
                        << " mean_identity_hard_sym_m="
                        << meanIdentityHardSymM
                        
                        << " mean_csgn_easy_sym_m="
                        << meanCsgnEasySymM
                        
                        << " mean_identity_easy_sym_m="
                        << meanIdentityEasySymM
                        
                        << " mean_hard_preservation="
                        << meanHardPreservation
                        
                        << " mean_easy_preservation="
                        << meanEasyPreservation
                        
                        << " mean_preferential_preservation="
                        << meanPreferentialPreservation);

                    if (pairedPointWidthValidCount == 0)
                    {
                        minCsgnPointReferenceMarginM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    
                        minIdentityPointReferenceMarginM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    const double meanCsgnHardWidthM =
                        pairedPointWidthValidCount > 0
                            ? sumCsgnHardWidthM /
                                  static_cast<double>(
                                      pairedPointWidthValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanIdentityHardWidthM =
                        pairedPointWidthValidCount > 0
                            ? sumIdentityHardWidthM /
                                  static_cast<double>(
                                      pairedPointWidthValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double meanCsgnMiddleWidthM =
                        pairedPointWidthValidCount > 0
                            ? sumCsgnMiddleWidthM /
                                  static_cast<double>(
                                      pairedPointWidthValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanIdentityMiddleWidthM =
                        pairedPointWidthValidCount > 0
                            ? sumIdentityMiddleWidthM /
                                  static_cast<double>(
                                      pairedPointWidthValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double meanCsgnEasyWidthM =
                        pairedPointWidthValidCount > 0
                            ? sumCsgnEasyWidthM /
                                  static_cast<double>(
                                      pairedPointWidthValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanIdentityEasyWidthM =
                        pairedPointWidthValidCount > 0
                            ? sumIdentityEasyWidthM /
                                  static_cast<double>(
                                      pairedPointWidthValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    ROS_INFO_STREAM(
                        "TF_CSGN_IDENTITY_POINT_WIDTH_SUMMARY "
                    
                        << "valid="
                        << pairedPointWidthValidCount
                    
                        << " total="
                        << controlledSegmentCount
                    
                        << " min_csgn_reference_margin_m="
                        << minCsgnPointReferenceMarginM
                    
                        << " min_identity_reference_margin_m="
                        << minIdentityPointReferenceMarginM
                    
                        << " mean_csgn_hard_width_m="
                        << meanCsgnHardWidthM
                    
                        << " mean_identity_hard_width_m="
                        << meanIdentityHardWidthM
                    
                        << " mean_csgn_mid_width_m="
                        << meanCsgnMiddleWidthM
                    
                        << " mean_identity_mid_width_m="
                        << meanIdentityMiddleWidthM
                    
                        << " mean_csgn_easy_width_m="
                        << meanCsgnEasyWidthM
                    
                        << " mean_identity_easy_width_m="
                        << meanIdentityEasyWidthM);
                        
                    const double meanControlledCsgnVolumeM3 =
                        pairedVolumeValidCount > 0
                            ? sumControlledCsgnVolumeM3 /
                                  static_cast<double>(
                                      pairedVolumeValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    const double meanControlledIdentityVolumeM3 =
                        pairedVolumeValidCount > 0
                            ? sumControlledIdentityVolumeM3 /
                                  static_cast<double>(
                                      pairedVolumeValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                    if (pairedVolumeValidCount == 0)
                    {
                        maxVolumeVertexViolationM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }
                    
                    ROS_INFO_STREAM(
                        "TF_CORRIDOR_VOLUME_SUMMARY "
                    
                        << "selftest_valid="
                        << volumeSelfTestValid
                    
                        << " valid="
                        << pairedVolumeValidCount
                    
                        << " total="
                        << controlledSegmentCount
                    
                        << " mean_csgn_volume_m3="
                        << meanControlledCsgnVolumeM3
                    
                        << " mean_identity_volume_m3="
                        << meanControlledIdentityVolumeM3
                    
                        << " max_vertex_violation_m="
                        << maxVolumeVertexViolationM);

                    // ========================================================
                    // D1a-1: Controlled-Geometry STANDARD FIRI baseline.
                    //
                    // Protocol:
                    //
                    //   original route segment i
                    //       ->
                    //   exactly one standard FIRI polytope i
                    //
                    // Differences from native sfc_gen::convexCover():
                    //
                    //   - no progress-based sub-segmentation;
                    //   - no inserted gap polytope;
                    //   - no shortcut;
                    //
                    // Kept identical:
                    //
                    //   - local axis-aligned bounding domain;
                    //   - local obstacle crop;
                    //   - FIRI inner kernel;
                    //   - FIRI iteration count / epsilon.
                    //
                    // The Direct-MINCO CSGN metric is supplied ONLY for
                    // post-selection diagnostics.  metric_weight == 0 means
                    // it does NOT alter standard FIRI construction.
                    //
                    // Cross-method directional geometry is measured from the
                    // ORIGINAL segment MIDPOINT using the common
                    // evaluatePointDirectionalWidth() kernel.
                    //
                    // This block is after all frozen Proposed timers.
                    // ========================================================

                    std::vector<Eigen::MatrixX4d>
                        controlledFiriHPolys;

                    std::vector<
                        firi::TrajectoryFavorableDiagnostics>
                        controlledFiriDiagnostics;

                    std::vector<int>
                        controlledFiriLocalObstacleCounts;

                    std::vector<double>
                        controlledFiriSegmentMs;


                    controlledFiriHPolys.reserve(
                        std::max(
                            0,
                            controlledSegmentCount));
                        
                    controlledFiriDiagnostics.reserve(
                        std::max(
                            0,
                            controlledSegmentCount));
                        
                    controlledFiriLocalObstacleCounts.reserve(
                        std::max(
                            0,
                            controlledSegmentCount));
                        
                    controlledFiriSegmentMs.reserve(
                        std::max(
                            0,
                            controlledSegmentCount));
                        
                        
                    const double controlledFiriRangeM =
                        config.tfFiriRange;
                        
                    const int controlledFiriIterations =
                        4;
                        
                    const double controlledFiriEpsilon =
                        1.0e-6;
                        
                        
                    // --------------------------------------------------------
                    // Controlled one-original-segment -> one-FIRI-polytope
                    // builder.
                    // --------------------------------------------------------
                    auto buildControlledFiri =
                        [&]() -> bool
                    {
                        controlledFiriHPolys.clear();
                        controlledFiriDiagnostics.clear();
                        controlledFiriLocalObstacleCounts.clear();
                        controlledFiriSegmentMs.clear();
                    
                        if (controlledSegmentCount <= 0 ||
                            static_cast<int>(
                                route.size()) !=
                                controlledSegmentCount + 1 ||
                            !std::isfinite(
                                controlledFiriRangeM) ||
                            controlledFiriRangeM <= 0.0)
                        {
                            return false;
                        }
                    
                    
                        const Eigen::Vector3d lowCorner =
                            voxelMap.getOrigin();
                    
                        const Eigen::Vector3d highCorner =
                            voxelMap.getCorner();
                    
                    
                        for (int segmentId = 0;
                             segmentId <
                                 controlledSegmentCount;
                             ++segmentId)
                        {
                            const Eigen::Vector3d &a =
                                route[segmentId];
                        
                            const Eigen::Vector3d &b =
                                route[segmentId + 1];
                        
                        
                            // ====================================================
                            // Same six-face axis-aligned local bounding domain
                            // used by sfc_gen::convexCover().
                            // ====================================================
                            Eigen::Matrix<double, 6, 4> bd =
                                Eigen::Matrix<double, 6, 4>::
                                    Zero();
                        
                            bd(0, 0) = 1.0;
                            bd(1, 0) = -1.0;
                        
                            bd(2, 1) = 1.0;
                            bd(3, 1) = -1.0;
                        
                            bd(4, 2) = 1.0;
                            bd(5, 2) = -1.0;
                        
                        
                            bd(0, 3) =
                                -std::min(
                                    std::max(
                                        a(0),
                                        b(0)) +
                                        controlledFiriRangeM,
                                    highCorner(0));
                                    
                            bd(1, 3) =
                                +std::max(
                                    std::min(
                                        a(0),
                                        b(0)) -
                                        controlledFiriRangeM,
                                    lowCorner(0));
                                    
                            bd(2, 3) =
                                -std::min(
                                    std::max(
                                        a(1),
                                        b(1)) +
                                        controlledFiriRangeM,
                                    highCorner(1));
                                    
                            bd(3, 3) =
                                +std::max(
                                    std::min(
                                        a(1),
                                        b(1)) -
                                        controlledFiriRangeM,
                                    lowCorner(1));
                                    
                            bd(4, 3) =
                                -std::min(
                                    std::max(
                                        a(2),
                                        b(2)) +
                                        controlledFiriRangeM,
                                    highCorner(2));
                                    
                            bd(5, 3) =
                                +std::max(
                                    std::min(
                                        a(2),
                                        b(2)) -
                                        controlledFiriRangeM,
                                    lowCorner(2));
                                    
                                    
                            // ====================================================
                            // Same local point-cloud crop as convexCover().
                            //
                            // Do NOT use Eigen::Map(valid_pc[0]) here because
                            // an empty local cloud would make that undefined.
                            // ====================================================
                            std::vector<Eigen::Vector3d>
                                localPoints;
                                    
                            localPoints.reserve(
                                pc.size());
                            
                            
                            for (const Eigen::Vector3d &point :
                                 pc)
                            {
                                if ((
                                        bd.leftCols<3>() *
                                            point +
                                        bd.rightCols<1>())
                                        .maxCoeff() <
                                    0.0)
                                {
                                    localPoints.push_back(
                                        point);
                                }
                            }
                        
                        
                            Eigen::Matrix3Xd localPc(
                                3,
                                static_cast<int>(
                                    localPoints.size()));
                                
                                
                            for (int pointId = 0;
                                 pointId <
                                     static_cast<int>(
                                         localPoints.size());
                                 ++pointId)
                            {
                                localPc.col(
                                    pointId) =
                                    localPoints[
                                        pointId];
                            }
                        
                        
                            // ====================================================
                            // STANDARD FIRI.
                            //
                            // All construction-conditioning weights are zero.
                            //
                            // metric_enabled may be true so final selected faces
                            // can still be evaluated against the SAME Direct-MINCO
                            // CSGN utility, but metric_weight remains zero.
                            // ====================================================
                            firi::TrajectoryFavorableOptions
                                firiOptions;
                        
                            firiOptions.enabled =
                                false;
                        
                            firiOptions.directional_width_weight =
                                0.0;
                        
                            firiOptions.face_count_weight =
                                0.0;
                        
                            firiOptions.max_faces =
                                0;
                        
                            firiOptions.metric_enabled =
                                false;
                        
                            firiOptions.metric_weight =
                                0.0;
                        
                        
                            if (segmentId <
                                    static_cast<int>(
                                        guideSegmentMetrics.size()) &&
                                guideSegmentMetrics[
                                    segmentId]
                                    .valid &&
                                guideSegmentMetrics[
                                    segmentId]
                                    .utility
                                    .allFinite())
                            {
                                firiOptions.metric_enabled =
                                    true;
                            
                                firiOptions.deformation_utility =
                                    guideSegmentMetrics[
                                        segmentId]
                                        .utility;
                            }
                        
                        
                            Eigen::MatrixX4d hPoly;
                        
                            firi::TrajectoryFavorableDiagnostics
                                diagnostics;
                        
                        
                            const auto segmentStarted =
                                std::chrono::
                                    steady_clock::now();
                        
                        
                            const bool generated =
                                firi::firi(
                                    bd,
                                    localPc,
                                    a,
                                    b,
                                    hPoly,
                                    controlledFiriIterations,
                                    controlledFiriEpsilon,
                                    firiOptions,
                                    &diagnostics);
                                
                                
                            const double segmentMs =
                                std::chrono::duration<
                                    double,
                                    std::milli>(
                                        std::chrono::
                                            steady_clock::now() -
                                        segmentStarted)
                                    .count();
                                    
                                    
                            controlledFiriDiagnostics
                                .push_back(
                                    diagnostics);
                                
                            controlledFiriLocalObstacleCounts
                                .push_back(
                                    static_cast<int>(
                                        localPoints.size()));
                                    
                            controlledFiriSegmentMs
                                .push_back(
                                    segmentMs);
                                
                                
                            if (!generated ||
                                hPoly.rows() <= 0 ||
                                !hPoly.allFinite())
                            {
                                return false;
                            }
                        
                        
                            controlledFiriHPolys
                                .push_back(
                                    hPoly);
                        }
                    
                    
                        return
                            static_cast<int>(
                                controlledFiriHPolys.size()) ==
                                controlledSegmentCount &&
                            static_cast<int>(
                                controlledFiriDiagnostics.size()) ==
                                controlledSegmentCount &&
                            static_cast<int>(
                                controlledFiriLocalObstacleCounts.size()) ==
                                controlledSegmentCount &&
                            static_cast<int>(
                                controlledFiriSegmentMs.size()) ==
                                controlledSegmentCount;
                    };


                    // --------------------------------------------------------
                    // Build controlled standard FIRI.
                    //
                    // This wall time is diagnostic only and is outside the
                    // frozen Proposed after_route_ms.
                    // --------------------------------------------------------
                    const auto controlledFiriStarted =
                        std::chrono::
                            steady_clock::now();


                    const bool controlledFiriSuccess =
                        guideSegmentMetricsReady &&
                        buildControlledFiri();


                    const double controlledFiriGenerationMs =
                        std::chrono::duration<
                            double,
                            std::milli>(
                                std::chrono::
                                    steady_clock::now() -
                                controlledFiriStarted)
                            .count();
                            
                            
                    const bool controlledFiriMappingValid =
                        controlledFiriSuccess &&
                        static_cast<int>(
                            controlledFiriHPolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledFiriDiagnostics.size()) ==
                            controlledSegmentCount;
                        
                        
                    // ========================================================
                    // Controlled-FIRI common geometry measurements.
                    // ========================================================
                    int controlledFiriSeedValidCount =
                        0;
                        
                    int controlledFiriOverlapValidCount =
                        0;
                        
                    int controlledFiriDirectionalValidCount =
                        0;
                        
                    int controlledFiriVolumeValidCount =
                        0;
                        
                        
                    int controlledFiriTotalFaces =
                        0;
                        
                    int controlledFiriDomainFaces =
                        0;
                        
                    int controlledFiriObstacleFaces =
                        0;
                        
                    int controlledFiriTotalLocalObstacles =
                        0;
                        
                    int controlledFiriUnresolvedConstraints =
                        0;
                        
                        
                    double controlledFiriMinSeedRadiusM =
                        std::numeric_limits<double>::
                            infinity();
                        
                    double controlledFiriMinOverlapRadiusM =
                        std::numeric_limits<double>::
                            infinity();
                        
                    double controlledFiriMinReferenceMarginM =
                        std::numeric_limits<double>::
                            infinity();
                        
                    double controlledFiriHardWidthSumM =
                        0.0;
                        
                    double controlledFiriMiddleWidthSumM =
                        0.0;
                        
                    double controlledFiriEasyWidthSumM =
                        0.0;
                        
                    double controlledFiriVolumeSumM3 =
                        0.0;
                        
                    double controlledFiriMaxVertexViolationM =
                        -std::numeric_limits<double>::
                            infinity();
                        
                        
                    if (controlledFiriMappingValid)
                    {
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            const Eigen::MatrixX4d &firiPoly =
                                controlledFiriHPolys[
                                    corridorId];
                                
                            const firi::
                                TrajectoryFavorableDiagnostics
                                    &firiDiagnostics =
                                        controlledFiriDiagnostics[
                                            corridorId];
                                        
                                        
                            // ====================================================
                            // This is the ONLY seedA / seedB pair in the new
                            // D1a-1 block.
                            //
                            // Cross-method directional width does NOT use the
                            // protected capsule.  Its fixed reference is the
                            // ORIGINAL route-segment midpoint.
                            // ====================================================
                            const Eigen::Vector3d &seedA =
                                route[corridorId];
                                        
                            const Eigen::Vector3d &seedB =
                                route[corridorId + 1];
                                        
                            const Eigen::Vector3d
                                directionalReferencePoint =
                                    0.5 *
                                    (
                                        seedA +
                                        seedB);
                                    
                                    
                            // ====================================================
                            // C2 geometry:
                            // segment seed radius.
                            // ====================================================
                            const auto seedMetric =
                                gcopter_benchmark::
                                    evaluateSegmentSeedRadius(
                                        firiPoly,
                                        seedA,
                                        seedB);
                                    
                                    
                            if (seedMetric.valid)
                            {
                                ++controlledFiriSeedValidCount;
                            
                                controlledFiriMinSeedRadiusM =
                                    std::min(
                                        controlledFiriMinSeedRadiusM,
                                        seedMetric.radius_m);
                            }
                        
                        
                            // ====================================================
                            // C2 geometry:
                            // fixed route-junction overlap radius.
                            // ====================================================
                            gcopter_benchmark::
                                CorridorOverlapMetric
                                    overlapMetric;
                        
                        
                            if (corridorId + 1 <
                                controlledSegmentCount)
                            {
                                overlapMetric =
                                    gcopter_benchmark::
                                        evaluateJunctionOverlapRadius(
                                            controlledFiriHPolys[
                                                corridorId],
                                            controlledFiriHPolys[
                                                corridorId + 1],
                                            route[corridorId + 1]);
                                            
                                            
                                if (overlapMetric.valid)
                                {
                                    ++controlledFiriOverlapValidCount;
                                
                                    controlledFiriMinOverlapRadiusM =
                                        std::min(
                                            controlledFiriMinOverlapRadiusM,
                                            overlapMetric.radius_m);
                                }
                            }
                        
                        
                            // ====================================================
                            // Secondary volume metric.
                            // ====================================================
                            const auto volumeMetric =
                                gcopter_benchmark::
                                    evaluateHPolytopeVolume(
                                        firiPoly);
                                    
                                    
                            if (volumeMetric.valid)
                            {
                                ++controlledFiriVolumeValidCount;
                            
                                controlledFiriVolumeSumM3 +=
                                    volumeMetric.volume_m3;
                            
                                controlledFiriMaxVertexViolationM =
                                    std::max(
                                        controlledFiriMaxVertexViolationM,
                                        volumeMetric
                                            .max_vertex_violation_m);
                            }
                        
                        
                            // ====================================================
                            // Common Direct-MINCO CSGN reference directions.
                            //
                            // IMPORTANT:
                            // These directions measure FIRI; they do NOT alter
                            // FIRI construction.
                            // ====================================================
                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    hardWidth;
                        
                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    middleWidth;
                        
                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    easyWidth;
                        
                        
                            bool directionalValid =
                                false;
                        
                        
                            if (corridorId <
                                    static_cast<int>(
                                        guideSegmentMetrics.size()) &&
                                guideSegmentMetrics[
                                    corridorId]
                                    .valid)
                            {
                                Eigen::Matrix3d utility =
                                    guideSegmentMetrics[
                                        corridorId]
                                        .utility;
                                    
                                    
                                utility =
                                    0.5 *
                                    (
                                        utility +
                                        utility.transpose());
                                    
                                    
                                Eigen::SelfAdjointEigenSolver<
                                    Eigen::Matrix3d>
                                    utilitySolver(
                                        utility);
                                    
                                    
                                if (utilitySolver.info() ==
                                        Eigen::Success &&
                                    utilitySolver
                                            .eigenvalues()
                                            .minCoeff() >
                                        0.0)
                                {
                                    Eigen::Matrix3d directions =
                                        utilitySolver
                                            .eigenvectors();
                                
                                
                                    // Same sign canonicalization used by
                                    // C2c/C2d.
                                    for (int directionId = 0;
                                         directionId < 3;
                                         ++directionId)
                                    {
                                        Eigen::Vector3d direction =
                                            directions.col(
                                                directionId);
                                            
                                        Eigen::Index pivotId =
                                            0;
                                            
                                        direction
                                            .cwiseAbs()
                                            .maxCoeff(
                                                &pivotId);
                                            
                                            
                                        if (direction(
                                                pivotId) <
                                            0.0)
                                        {
                                            direction =
                                                -direction;
                                        }
                                    
                                    
                                        directions.col(
                                            directionId) =
                                            direction;
                                    }
                                
                                
                                    hardWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                firiPoly,
                                                directionalReferencePoint,
                                                directions.col(0));
                                            
                                            
                                    middleWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                firiPoly,
                                                directionalReferencePoint,
                                                directions.col(1));
                                            
                                            
                                    easyWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                firiPoly,
                                                directionalReferencePoint,
                                                directions.col(2));
                                            
                                            
                                    directionalValid =
                                        hardWidth.valid &&
                                        middleWidth.valid &&
                                        easyWidth.valid &&
                                        hardWidth.reference_inside &&
                                        middleWidth.reference_inside &&
                                        easyWidth.reference_inside;
                                            
                                            
                                    if (directionalValid)
                                    {
                                        ++controlledFiriDirectionalValidCount;
                                    
                                        controlledFiriHardWidthSumM +=
                                            hardWidth.width_m;
                                    
                                        controlledFiriMiddleWidthSumM +=
                                            middleWidth.width_m;
                                    
                                        controlledFiriEasyWidthSumM +=
                                            easyWidth.width_m;
                                    
                                        controlledFiriMinReferenceMarginM =
                                            std::min(
                                                controlledFiriMinReferenceMarginM,
                                                hardWidth
                                                    .min_reference_margin_m);
                                    }
                                }
                            }
                        
                        
                            // ====================================================
                            // Face / local-construction workload.
                            // ====================================================
                            const int totalFaces =
                                static_cast<int>(
                                    firiPoly.rows());
                                
                            const int obstacleFaces =
                                firiDiagnostics
                                    .obstacle_face_count;
                                
                            const int domainFaces =
                                std::max(
                                    0,
                                    totalFaces -
                                        obstacleFaces);
                                
                                
                            controlledFiriTotalFaces +=
                                totalFaces;
                                
                            controlledFiriDomainFaces +=
                                domainFaces;
                                
                            controlledFiriObstacleFaces +=
                                obstacleFaces;
                                
                            controlledFiriTotalLocalObstacles +=
                                controlledFiriLocalObstacleCounts[
                                    corridorId];
                                
                            controlledFiriUnresolvedConstraints +=
                                firiDiagnostics
                                    .unresolved_constraint_count;
                                
                                
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_FIRI_GEOMETRY "
                            
                                << "corridor_id="
                                << corridorId
                            
                                << " generated=1"
                            
                                << " seed_valid="
                                << seedMetric.valid
                            
                                << " seed_radius_m="
                                << seedMetric.radius_m
                            
                                << " junction_overlap_valid="
                                << overlapMetric.valid
                            
                                << " junction_overlap_radius_m="
                                << overlapMetric.radius_m
                            
                                << " directional_valid="
                                << directionalValid
                            
                                << " reference_margin_m="
                                << hardWidth
                                       .min_reference_margin_m
                            
                                << " hard_pos_m="
                                << hardWidth.positive_m
                            
                                << " hard_neg_m="
                                << hardWidth.negative_m
                            
                                << " hard_width_m="
                                << hardWidth.width_m
                            
                                << " mid_pos_m="
                                << middleWidth.positive_m
                            
                                << " mid_neg_m="
                                << middleWidth.negative_m
                            
                                << " mid_width_m="
                                << middleWidth.width_m
                            
                                << " easy_pos_m="
                                << easyWidth.positive_m
                            
                                << " easy_neg_m="
                                << easyWidth.negative_m
                            
                                << " easy_width_m="
                                << easyWidth.width_m
                            
                                << " volume_valid="
                                << volumeMetric.valid
                            
                                << " volume_m3="
                                << volumeMetric.volume_m3
                            
                                << " volume_vertices="
                                << volumeMetric.vertex_count
                            
                                << " volume_triangles="
                                << volumeMetric.triangle_count
                            
                                << " vertex_violation_m="
                                << volumeMetric
                                       .max_vertex_violation_m
                            
                                << " total_faces="
                                << totalFaces
                            
                                << " domain_faces="
                                << domainFaces
                            
                                << " obstacle_faces="
                                << obstacleFaces
                            
                                << " local_obstacles="
                                << controlledFiriLocalObstacleCounts[
                                       corridorId]
                                
                                << " unresolved_constraints="
                                << firiDiagnostics
                                       .unresolved_constraint_count
                                
                                << " unresolved_boundary="
                                << firiDiagnostics
                                       .unresolved_boundary_count
                                
                                << " unresolved_obstacle="
                                << firiDiagnostics
                                       .unresolved_obstacle_count
                                
                                << " mean_metric_damage="
                                << firiDiagnostics
                                       .mean_metric_damage
                                
                                << " min_metric_damage="
                                << firiDiagnostics
                                       .min_metric_damage
                                
                                << " max_metric_damage="
                                << firiDiagnostics
                                       .max_metric_damage
                                
                                << " generation_ms="
                                << controlledFiriSegmentMs[
                                       corridorId]);
                        }
                    }


                    // ========================================================
                    // Aggregate Controlled-FIRI regression summary.
                    // ========================================================
                    if (controlledFiriSeedValidCount == 0)
                    {
                        controlledFiriMinSeedRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    if (controlledFiriOverlapValidCount == 0)
                    {
                        controlledFiriMinOverlapRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    if (controlledFiriDirectionalValidCount == 0)
                    {
                        controlledFiriMinReferenceMarginM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    if (controlledFiriVolumeValidCount == 0)
                    {
                        controlledFiriMaxVertexViolationM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    const double controlledFiriMeanHardWidthM =
                        controlledFiriDirectionalValidCount > 0
                            ? controlledFiriHardWidthSumM /
                                  static_cast<double>(
                                      controlledFiriDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double controlledFiriMeanMiddleWidthM =
                        controlledFiriDirectionalValidCount > 0
                            ? controlledFiriMiddleWidthSumM /
                                  static_cast<double>(
                                      controlledFiriDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double controlledFiriMeanEasyWidthM =
                        controlledFiriDirectionalValidCount > 0
                            ? controlledFiriEasyWidthSumM /
                                  static_cast<double>(
                                      controlledFiriDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double controlledFiriMeanVolumeM3 =
                        controlledFiriVolumeValidCount > 0
                            ? controlledFiriVolumeSumM3 /
                                  static_cast<double>(
                                      controlledFiriVolumeValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_FIRI_SUMMARY "
                    
                        << "success="
                        << controlledFiriSuccess
                    
                        << " mapping_valid="
                        << controlledFiriMappingValid
                    
                        << " route_segments="
                        << controlledSegmentCount
                    
                        << " corridors="
                        << controlledFiriHPolys.size()
                    
                        << " seed_valid="
                        << controlledFiriSeedValidCount
                    
                        << " seed_total="
                        << controlledSegmentCount
                    
                        << " min_seed_m="
                        << controlledFiriMinSeedRadiusM
                    
                        << " overlap_valid="
                        << controlledFiriOverlapValidCount
                    
                        << " overlap_total="
                        << std::max(
                               0,
                               controlledSegmentCount - 1)
                        
                        << " min_overlap_m="
                        << controlledFiriMinOverlapRadiusM
                        
                        << " directional_valid="
                        << controlledFiriDirectionalValidCount
                        
                        << " directional_total="
                        << controlledSegmentCount
                        
                        << " min_reference_margin_m="
                        << controlledFiriMinReferenceMarginM
                        
                        << " mean_hard_width_m="
                        << controlledFiriMeanHardWidthM
                        
                        << " mean_mid_width_m="
                        << controlledFiriMeanMiddleWidthM
                        
                        << " mean_easy_width_m="
                        << controlledFiriMeanEasyWidthM
                        
                        << " volume_valid="
                        << controlledFiriVolumeValidCount
                        
                        << " volume_total="
                        << controlledSegmentCount
                        
                        << " mean_volume_m3="
                        << controlledFiriMeanVolumeM3
                        
                        << " max_vertex_violation_m="
                        << controlledFiriMaxVertexViolationM
                        
                        << " total_faces="
                        << controlledFiriTotalFaces
                        
                        << " domain_faces="
                        << controlledFiriDomainFaces
                        
                        << " obstacle_faces="
                        << controlledFiriObstacleFaces
                        
                        << " total_local_obstacles="
                        << controlledFiriTotalLocalObstacles
                        
                        << " unresolved_constraints="
                        << controlledFiriUnresolvedConstraints
                        
                        << " range_m="
                        << controlledFiriRangeM
                        
                        << " iterations="
                        << controlledFiriIterations
                        
                        << " generation_ms="
                        << controlledFiriGenerationMs);

                    // ========================================================
                    // D1b-0: Controlled Liu/RILS reference builder.
                    //
                    // Construction only in this stage.
                    //
                    // Geometry metrics are intentionally deferred to D1b-1.
                    // This keeps construction correctness separate from
                    // cross-method measurement.
                    //
                    // Exactly:
                    //
                    //   one ORIGINAL route segment
                    //       ->
                    //   one DecompUtil LineSegment3D
                    //       ->
                    //   one RILS polyhedron.
                    //
                    // Controlled local range is 3 m, shared with the
                    // Controlled FIRI domain budget.
                    //
                    // This is post-timing and cannot modify frozen
                    // Proposed after_route_ms.
                    // ========================================================
                                            
                    const double controlledRilsRangeM =
                        controlledFiriRangeM;
                                            
                                            
                    const auto controlledRilsBuild =
                        gcopter_benchmark::
                            buildControlledRilsCorridors(
                                route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                controlledRilsRangeM);
                            
                            
                    int controlledRilsValidCount =
                        0;
                            
                    int controlledRilsGeneratorFaces =
                        0;
                            
                    int controlledRilsObstacleFaces =
                        0;
                            
                    int controlledRilsLocalDomainFaces =
                        0;
                            
                    int controlledRilsMapDomainFaces =
                        0;
                            
                    int controlledRilsFinalRows =
                        0;
                            
                    int controlledRilsLocalObstacles =
                        0;
                            
                    double controlledRilsMaxSeedViolationM =
                        0.0;
                            
                    double controlledRilsDilateMs =
                        0.0;
                            
                            
                    for (const auto &info :
                         controlledRilsBuild.infos)
                    {
                        if (info.valid)
                        {
                            ++controlledRilsValidCount;
                        }
                    
                    
                        controlledRilsGeneratorFaces +=
                            info.generator_faces;
                    
                        controlledRilsObstacleFaces +=
                            info.obstacle_faces;
                    
                        controlledRilsLocalDomainFaces +=
                            info.local_domain_faces;
                    
                        controlledRilsMapDomainFaces +=
                            info.map_domain_faces;
                    
                        controlledRilsFinalRows +=
                            info.final_rows;
                    
                        controlledRilsLocalObstacles +=
                            info.local_obstacle_count;
                    
                    
                        if (std::isfinite(
                                info.seed_max_violation_m))
                        {
                            controlledRilsMaxSeedViolationM =
                                std::max(
                                    controlledRilsMaxSeedViolationM,
                                    info.seed_max_violation_m);
                        }
                    
                    
                        if (std::isfinite(
                                info.dilate_ms))
                        {
                            controlledRilsDilateMs +=
                                info.dilate_ms;
                        }
                    
                    
                        ROS_INFO_STREAM(
                            "TF_CONTROLLED_RILS_BUILD "
                        
                            << "corridor_id="
                            << info.corridor_id
                        
                            << " valid="
                            << info.valid
                        
                            << " local_obstacles="
                            << info.local_obstacle_count
                        
                            << " generator_faces="
                            << info.generator_faces
                        
                            << " obstacle_faces="
                            << info.obstacle_faces
                        
                            << " local_domain_faces="
                            << info.local_domain_faces
                        
                            << " map_domain_faces="
                            << info.map_domain_faces
                        
                            << " final_rows="
                            << info.final_rows
                        
                            << " seed_max_violation_m="
                            << info.seed_max_violation_m
                        
                            << " dilate_ms="
                            << info.dilate_ms);
                    }
                    
                    
                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_RILS_BUILD_SUMMARY "
                    
                        << "available="
                        << controlledRilsBuild.available
                    
                        << " success="
                        << controlledRilsBuild.success
                    
                        << " mapping_valid="
                        << controlledRilsBuild.mapping_valid
                    
                        << " route_segments="
                        << controlledSegmentCount
                    
                        << " corridors="
                        << controlledRilsBuild.hpolys.size()
                    
                        << " valid="
                        << controlledRilsValidCount
                    
                        << " valid_total="
                        << controlledSegmentCount
                    
                        << " range_m="
                        << controlledRilsBuild.range_m
                    
                        << " generator_faces="
                        << controlledRilsGeneratorFaces
                    
                        << " obstacle_faces="
                        << controlledRilsObstacleFaces
                    
                        << " local_domain_faces="
                        << controlledRilsLocalDomainFaces
                    
                        << " map_domain_faces="
                        << controlledRilsMapDomainFaces
                    
                        << " final_rows="
                        << controlledRilsFinalRows
                    
                        << " local_obstacles="
                        << controlledRilsLocalObstacles
                    
                        << " max_seed_violation_m="
                        << controlledRilsMaxSeedViolationM
                    
                        << " dilate_ms="
                        << controlledRilsDilateMs
                    
                        << " adapter_total_ms="
                        << controlledRilsBuild.total_ms);

                    // ========================================================
                    // D1b-1: Common E1 geometry evaluation for Controlled RILS.
                    //
                    // Construction was frozen in D1b-0.
                    //
                    // This stage measures the resulting RILS H-polytopes using
                    // exactly the same common kernels already used for:
                    //   - Controlled CSGN
                    //   - Controlled Identity
                    //   - Controlled FIRI
                    //
                    // Directional geometry:
                    //
                    //   q_i = midpoint of ORIGINAL route segment i
                    //
                    // and directions are the SAME Direct-MINCO CSGN
                    // eigendirections:
                    //
                    //   eig0 = hard / low-utility
                    //   eig1 = middle
                    //   eig2 = easy / high-utility
                    //
                    // No RILS construction parameter is changed here.
                    // ========================================================

                    int controlledRilsSeedValidCount =
                        0;

                    int controlledRilsOverlapValidCount =
                        0;

                    int controlledRilsDirectionalValidCount =
                        0;

                    int controlledRilsVolumeValidCount =
                        0;


                    double controlledRilsMinSeedRadiusM =
                        std::numeric_limits<double>::
                            infinity();

                    double controlledRilsMinOverlapRadiusM =
                        std::numeric_limits<double>::
                            infinity();

                    double controlledRilsMinReferenceMarginM =
                        std::numeric_limits<double>::
                            infinity();

                    double controlledRilsHardWidthSumM =
                        0.0;

                    double controlledRilsMiddleWidthSumM =
                        0.0;

                    double controlledRilsEasyWidthSumM =
                        0.0;

                    double controlledRilsVolumeSumM3 =
                        0.0;

                    double controlledRilsMaxVertexViolationM =
                        -std::numeric_limits<double>::
                            infinity();


                    const bool controlledRilsGeometryMappingValid =
                        controlledRilsBuild.success &&
                        controlledRilsBuild.mapping_valid &&
                        static_cast<int>(
                            controlledRilsBuild.hpolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledRilsBuild.infos.size()) ==
                            controlledSegmentCount;
                        
                        
                    if (controlledRilsGeometryMappingValid)
                    {
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            const Eigen::MatrixX4d &rilsPoly =
                                controlledRilsBuild.hpolys[
                                    corridorId];
                                
                            const auto &rilsInfo =
                                controlledRilsBuild.infos[
                                    corridorId];
                                
                                
                            // ====================================================
                            // Fixed original route segment.
                            //
                            // This seedA/seedB exists ONLY inside the D1b-1
                            // Controlled-RILS geometry block.
                            // ====================================================
                            const Eigen::Vector3d &seedA =
                                route[corridorId];
                                
                            const Eigen::Vector3d &seedB =
                                route[corridorId + 1];
                                
                                
                            const Eigen::Vector3d
                                directionalReferencePoint =
                                    0.5 *
                                    (
                                        seedA +
                                        seedB);
                                    
                                    
                            // ====================================================
                            // C2:
                            // exact segment seed radius inside this H-polytope.
                            // ====================================================
                            const auto seedMetric =
                                gcopter_benchmark::
                                    evaluateSegmentSeedRadius(
                                        rilsPoly,
                                        seedA,
                                        seedB);
                                    
                                    
                            if (seedMetric.valid)
                            {
                                ++controlledRilsSeedValidCount;
                            
                                controlledRilsMinSeedRadiusM =
                                    std::min(
                                        controlledRilsMinSeedRadiusM,
                                        seedMetric.radius_m);
                            }
                        
                        
                            // ====================================================
                            // C2:
                            // fixed junction overlap radius with next corridor.
                            // ====================================================
                            gcopter_benchmark::
                                CorridorOverlapMetric
                                    overlapMetric;
                        
                        
                            if (corridorId + 1 <
                                controlledSegmentCount)
                            {
                                overlapMetric =
                                    gcopter_benchmark::
                                        evaluateJunctionOverlapRadius(
                                            controlledRilsBuild.hpolys[
                                                corridorId],
                                            controlledRilsBuild.hpolys[
                                                corridorId + 1],
                                            route[corridorId + 1]);
                                            
                                            
                                if (overlapMetric.valid)
                                {
                                    ++controlledRilsOverlapValidCount;
                                
                                    controlledRilsMinOverlapRadiusM =
                                        std::min(
                                            controlledRilsMinOverlapRadiusM,
                                            overlapMetric.radius_m);
                                }
                            }
                        
                        
                            // ====================================================
                            // Secondary volume metric.
                            //
                            // This is measured on the final stored H-polytope:
                            //
                            //   DecompUtil RILS planes
                            //       +
                            //   finite GCOPTER map-domain planes.
                            // ====================================================
                            const auto volumeMetric =
                                gcopter_benchmark::
                                    evaluateHPolytopeVolume(
                                        rilsPoly);
                                    
                                    
                            if (volumeMetric.valid)
                            {
                                ++controlledRilsVolumeValidCount;
                            
                                controlledRilsVolumeSumM3 +=
                                    volumeMetric.volume_m3;
                            
                                controlledRilsMaxVertexViolationM =
                                    std::max(
                                        controlledRilsMaxVertexViolationM,
                                        volumeMetric
                                            .max_vertex_violation_m);
                            }
                        
                        
                            // ====================================================
                            // Common cross-method point directional width.
                            //
                            // IMPORTANT:
                            // RILS' own ellipsoid axes are NOT used here.
                            //
                            // The measurement directions come from the SAME
                            // Direct-MINCO CSGN utility used by the other methods.
                            // ====================================================
                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    hardWidth;
                        
                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    middleWidth;
                        
                            gcopter_benchmark::
                                CorridorPointDirectionalWidthMetric
                                    easyWidth;
                        
                        
                            bool directionalValid =
                                false;
                        
                        
                            if (corridorId <
                                    static_cast<int>(
                                        guideSegmentMetrics.size()) &&
                                guideSegmentMetrics[
                                    corridorId]
                                    .valid)
                            {
                                Eigen::Matrix3d utility =
                                    guideSegmentMetrics[
                                        corridorId]
                                        .utility;
                                    
                                    
                                utility =
                                    0.5 *
                                    (
                                        utility +
                                        utility.transpose());
                                    
                                    
                                Eigen::SelfAdjointEigenSolver<
                                    Eigen::Matrix3d>
                                    utilitySolver(
                                        utility);
                                    
                                    
                                if (utilitySolver.info() ==
                                        Eigen::Success &&
                                    utilitySolver
                                            .eigenvalues()
                                            .minCoeff() >
                                        0.0)
                                {
                                    Eigen::Matrix3d directions =
                                        utilitySolver
                                            .eigenvectors();
                                
                                
                                    // Same deterministic sign convention used by
                                    // Controlled CSGN / Identity / FIRI.
                                    for (int directionId = 0;
                                         directionId < 3;
                                         ++directionId)
                                    {
                                        Eigen::Vector3d direction =
                                            directions.col(
                                                directionId);
                                            
                                        Eigen::Index pivotId =
                                            0;
                                            
                                        direction
                                            .cwiseAbs()
                                            .maxCoeff(
                                                &pivotId);
                                            
                                            
                                        if (direction(
                                                pivotId) <
                                            0.0)
                                        {
                                            direction =
                                                -direction;
                                        }
                                    
                                    
                                        directions.col(
                                            directionId) =
                                            direction;
                                    }
                                
                                
                                    hardWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                rilsPoly,
                                                directionalReferencePoint,
                                                directions.col(0));
                                            
                                            
                                    middleWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                rilsPoly,
                                                directionalReferencePoint,
                                                directions.col(1));
                                            
                                            
                                    easyWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                rilsPoly,
                                                directionalReferencePoint,
                                                directions.col(2));
                                            
                                            
                                    directionalValid =
                                        hardWidth.valid &&
                                        middleWidth.valid &&
                                        easyWidth.valid &&
                                            
                                        hardWidth.reference_inside &&
                                        middleWidth.reference_inside &&
                                        easyWidth.reference_inside;
                                            
                                            
                                    if (directionalValid)
                                    {
                                        ++controlledRilsDirectionalValidCount;
                                    
                                        controlledRilsHardWidthSumM +=
                                            hardWidth.width_m;
                                    
                                        controlledRilsMiddleWidthSumM +=
                                            middleWidth.width_m;
                                    
                                        controlledRilsEasyWidthSumM +=
                                            easyWidth.width_m;
                                    
                                    
                                        const double referenceMarginM =
                                            std::min(
                                                hardWidth
                                                    .min_reference_margin_m,
                                                std::min(
                                                    middleWidth
                                                        .min_reference_margin_m,
                                                    easyWidth
                                                        .min_reference_margin_m));
                                                
                                                
                                        controlledRilsMinReferenceMarginM =
                                            std::min(
                                                controlledRilsMinReferenceMarginM,
                                                referenceMarginM);
                                    }
                                }
                            }
                        
                        
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_RILS_GEOMETRY "
                            
                                << "corridor_id="
                                << corridorId
                            
                                << " generated="
                                << rilsInfo.valid
                            
                                << " seed_valid="
                                << seedMetric.valid
                            
                                << " seed_radius_m="
                                << seedMetric.radius_m
                            
                                << " junction_overlap_valid="
                                << overlapMetric.valid
                            
                                << " junction_overlap_radius_m="
                                << overlapMetric.radius_m
                            
                                << " directional_valid="
                                << directionalValid
                            
                                << " reference_margin_m="
                                << hardWidth
                                       .min_reference_margin_m
                            
                                << " hard_pos_m="
                                << hardWidth.positive_m
                            
                                << " hard_neg_m="
                                << hardWidth.negative_m
                            
                                << " hard_width_m="
                                << hardWidth.width_m
                            
                                << " mid_pos_m="
                                << middleWidth.positive_m
                            
                                << " mid_neg_m="
                                << middleWidth.negative_m
                            
                                << " mid_width_m="
                                << middleWidth.width_m
                            
                                << " easy_pos_m="
                                << easyWidth.positive_m
                            
                                << " easy_neg_m="
                                << easyWidth.negative_m
                            
                                << " easy_width_m="
                                << easyWidth.width_m
                            
                                << " volume_valid="
                                << volumeMetric.valid
                            
                                << " volume_m3="
                                << volumeMetric.volume_m3
                            
                                << " volume_vertices="
                                << volumeMetric.vertex_count
                            
                                << " volume_triangles="
                                << volumeMetric.triangle_count
                            
                                << " vertex_violation_m="
                                << volumeMetric
                                       .max_vertex_violation_m
                            
                                << " generator_faces="
                                << rilsInfo.generator_faces
                            
                                << " obstacle_faces="
                                << rilsInfo.obstacle_faces
                            
                                << " local_domain_faces="
                                << rilsInfo.local_domain_faces
                            
                                << " map_domain_faces="
                                << rilsInfo.map_domain_faces
                            
                                << " final_rows="
                                << rilsInfo.final_rows
                            
                                << " local_obstacles="
                                << rilsInfo.local_obstacle_count
                            
                                << " dilate_ms="
                                << rilsInfo.dilate_ms);
                        }
                    }


                    // ========================================================
                    // Aggregate Controlled-RILS common geometry summary.
                    // ========================================================
                    if (controlledRilsSeedValidCount == 0)
                    {
                        controlledRilsMinSeedRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    if (controlledRilsOverlapValidCount == 0)
                    {
                        controlledRilsMinOverlapRadiusM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    if (controlledRilsDirectionalValidCount == 0)
                    {
                        controlledRilsMinReferenceMarginM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    if (controlledRilsVolumeValidCount == 0)
                    {
                        controlledRilsMaxVertexViolationM =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    }


                    const double controlledRilsMeanHardWidthM =
                        controlledRilsDirectionalValidCount > 0
                            ? controlledRilsHardWidthSumM /
                                  static_cast<double>(
                                      controlledRilsDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double controlledRilsMeanMiddleWidthM =
                        controlledRilsDirectionalValidCount > 0
                            ? controlledRilsMiddleWidthSumM /
                                  static_cast<double>(
                                      controlledRilsDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double controlledRilsMeanEasyWidthM =
                        controlledRilsDirectionalValidCount > 0
                            ? controlledRilsEasyWidthSumM /
                                  static_cast<double>(
                                      controlledRilsDirectionalValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    const double controlledRilsMeanVolumeM3 =
                        controlledRilsVolumeValidCount > 0
                            ? controlledRilsVolumeSumM3 /
                                  static_cast<double>(
                                      controlledRilsVolumeValidCount)
                            : std::numeric_limits<double>::
                                  quiet_NaN();
                                
                                
                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_RILS_GEOMETRY_SUMMARY "
                    
                        << "success="
                        << controlledRilsBuild.success
                    
                        << " mapping_valid="
                        << controlledRilsGeometryMappingValid
                    
                        << " route_segments="
                        << controlledSegmentCount
                    
                        << " corridors="
                        << controlledRilsBuild.hpolys.size()
                    
                        << " seed_valid="
                        << controlledRilsSeedValidCount
                    
                        << " seed_total="
                        << controlledSegmentCount
                    
                        << " min_seed_m="
                        << controlledRilsMinSeedRadiusM
                    
                        << " overlap_valid="
                        << controlledRilsOverlapValidCount
                    
                        << " overlap_total="
                        << std::max(
                               0,
                               controlledSegmentCount - 1)
                        
                        << " min_overlap_m="
                        << controlledRilsMinOverlapRadiusM
                        
                        << " directional_valid="
                        << controlledRilsDirectionalValidCount
                        
                        << " directional_total="
                        << controlledSegmentCount
                        
                        << " min_reference_margin_m="
                        << controlledRilsMinReferenceMarginM
                        
                        << " mean_hard_width_m="
                        << controlledRilsMeanHardWidthM
                        
                        << " mean_mid_width_m="
                        << controlledRilsMeanMiddleWidthM
                        
                        << " mean_easy_width_m="
                        << controlledRilsMeanEasyWidthM
                        
                        << " volume_valid="
                        << controlledRilsVolumeValidCount
                        
                        << " volume_total="
                        << controlledSegmentCount
                        
                        << " mean_volume_m3="
                        << controlledRilsMeanVolumeM3
                        
                        << " max_vertex_violation_m="
                        << controlledRilsMaxVertexViolationM
                        
                        << " generator_faces="
                        << controlledRilsGeneratorFaces
                        
                        << " obstacle_faces="
                        << controlledRilsObstacleFaces
                        
                        << " local_domain_faces="
                        << controlledRilsLocalDomainFaces
                        
                        << " map_domain_faces="
                        << controlledRilsMapDomainFaces
                        
                        << " final_rows="
                        << controlledRilsFinalRows
                        
                        << " range_m="
                        << controlledRilsBuild.range_m);

                    // ========================================================
                    // D1c-1: Common effective-face measurement.
                    //
                    // IMPORTANT:
                    //
                    //   Measurement only.
                    //   No corridor is modified.
                    //
                    // All four controlled methods are evaluated by the SAME
                    // D1c-0 LP-based redundancy kernel:
                    //
                    //     raw H rows
                    //         ->
                    //     duplicate geometric-plane grouping
                    //         ->
                    //     leave-one-plane-group-out support LP
                    //         ->
                    //     effective / redundant geometric faces.
                    //
                    // Two quantities remain deliberately distinct:
                    //
                    //   raw_rows
                    //       actual stored / downstream H-constraint workload;
                    //
                    //   effective_faces
                    //       distinct nonredundant geometric boundary facets.
                    // ========================================================
                                            
                    struct ControlledEffectiveFaceSummary
                    {
                        bool mapping_valid =
                            false;
                    
                        int valid_corridors =
                            0;
                    
                        int total_corridors =
                            0;
                    
                        int raw_rows =
                            0;
                    
                        int unique_plane_groups =
                            0;
                    
                        int duplicate_rows =
                            0;
                    
                        int effective_faces =
                            0;
                    
                        int redundant_plane_groups =
                            0;
                    
                        int unbounded_support_tests =
                            0;
                    
                        double max_redundant_violation_m =
                            -std::numeric_limits<double>::
                                infinity();
                    
                        double min_finite_effective_violation_m =
                            std::numeric_limits<double>::
                                infinity();
                    };
                    
                    
                    // --------------------------------------------------------
                    // One common evaluator for every controlled method.
                    // --------------------------------------------------------
                    auto evaluateControlledEffectiveFaces =
                        [&](const std::string &methodName,
                            const std::vector<Eigen::MatrixX4d> &hPolys,
                            const bool mappingValid)
                            -> ControlledEffectiveFaceSummary
                    {
                        ControlledEffectiveFaceSummary summary;
                    
                        summary.mapping_valid =
                            mappingValid &&
                            static_cast<int>(
                                hPolys.size()) ==
                                controlledSegmentCount;
                            
                        summary.total_corridors =
                            controlledSegmentCount;
                            
                            
                        if (!summary.mapping_valid)
                        {
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_EFFECTIVE_FACE_SUMMARY "
                            
                                << "method="
                                << methodName
                            
                                << " mapping_valid=0"
                            
                                << " valid=0"
                            
                                << " total="
                                << controlledSegmentCount);
                            
                            return summary;
                        }
                    
                    
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            const auto metric =
                                gcopter_benchmark::
                                    evaluateEffectiveFaces(
                                        hPolys[
                                            corridorId]);
                                        
                                        
                            if (metric.valid)
                            {
                                ++summary.valid_corridors;
                            
                                summary.raw_rows +=
                                    metric.raw_rows;
                            
                                summary.unique_plane_groups +=
                                    metric.unique_plane_groups;
                            
                                summary.duplicate_rows +=
                                    metric.duplicate_rows;
                            
                                summary.effective_faces +=
                                    metric.effective_faces;
                            
                                summary.redundant_plane_groups +=
                                    metric.redundant_plane_groups;
                            
                                summary.unbounded_support_tests +=
                                    metric.unbounded_support_tests;
                            
                            
                                if (std::isfinite(
                                        metric
                                            .max_redundant_violation_m))
                                {
                                    summary.max_redundant_violation_m =
                                        std::max(
                                            summary
                                                .max_redundant_violation_m,
                                            metric
                                                .max_redundant_violation_m);
                                }
                            
                            
                                if (std::isfinite(
                                        metric
                                            .min_finite_effective_violation_m))
                                {
                                    summary
                                        .min_finite_effective_violation_m =
                                        std::min(
                                            summary
                                                .min_finite_effective_violation_m,
                                            metric
                                                .min_finite_effective_violation_m);
                                }
                            }
                        
                        
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_EFFECTIVE_FACE "
                            
                                << "method="
                                << methodName
                            
                                << " corridor_id="
                                << corridorId
                            
                                << " valid="
                                << metric.valid
                            
                                << " raw_rows="
                                << metric.raw_rows
                            
                                << " unique_groups="
                                << metric
                                       .unique_plane_groups
                            
                                << " duplicate_rows="
                                << metric
                                       .duplicate_rows
                            
                                << " effective_faces="
                                << metric
                                       .effective_faces
                            
                                << " redundant_groups="
                                << metric
                                       .redundant_plane_groups
                            
                                << " unbounded_tests="
                                << metric
                                       .unbounded_support_tests
                            
                                << " max_redundant_violation_m="
                                << metric
                                       .max_redundant_violation_m
                            
                                << " min_finite_effective_violation_m="
                                << metric
                                       .min_finite_effective_violation_m);
                        }
                    
                    
                        if (!std::isfinite(
                                summary
                                    .max_redundant_violation_m))
                        {
                            summary
                                .max_redundant_violation_m =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        }
                    
                    
                        if (!std::isfinite(
                                summary
                                    .min_finite_effective_violation_m))
                        {
                            summary
                                .min_finite_effective_violation_m =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        }
                    
                    
                        ROS_INFO_STREAM(
                            "TF_CONTROLLED_EFFECTIVE_FACE_SUMMARY "
                        
                            << "method="
                            << methodName
                        
                            << " mapping_valid="
                            << summary.mapping_valid
                        
                            << " valid="
                            << summary.valid_corridors
                        
                            << " total="
                            << summary.total_corridors
                        
                            << " raw_rows="
                            << summary.raw_rows
                        
                            << " unique_groups="
                            << summary
                                   .unique_plane_groups
                        
                            << " duplicate_rows="
                            << summary
                                   .duplicate_rows
                        
                            << " effective_faces="
                            << summary
                                   .effective_faces
                        
                            << " redundant_groups="
                            << summary
                                   .redundant_plane_groups
                        
                            << " unbounded_tests="
                            << summary
                                   .unbounded_support_tests
                        
                            << " max_redundant_violation_m="
                            << summary
                                   .max_redundant_violation_m
                        
                            << " min_finite_effective_violation_m="
                            << summary
                                   .min_finite_effective_violation_m);
                        
                        
                        return summary;
                    };
                    
                    
                    // ========================================================
                    // Evaluate all four methods.
                    //
                    // The order is fixed for deterministic regression logs.
                    // ========================================================
                    
                    const auto controlledCsgnEffectiveFaces =
                        evaluateControlledEffectiveFaces(
                            "csgn",
                            controlledCsgnHPolys,
                            controlledPairMappingValid);
                        
                        
                    const auto controlledIdentityEffectiveFaces =
                        evaluateControlledEffectiveFaces(
                            "identity",
                            controlledIdentityHPolys,
                            controlledPairMappingValid);
                        
                        
                    const auto controlledFiriEffectiveFaces =
                        evaluateControlledEffectiveFaces(
                            "firi",
                            controlledFiriHPolys,
                            controlledFiriMappingValid);
                        
                        
                    const auto controlledRilsEffectiveFaces =
                        evaluateControlledEffectiveFaces(
                            "rils",
                            controlledRilsBuild.hpolys,
                            controlledRilsGeometryMappingValid);
                        
                        
                    // --------------------------------------------------------
                    // One compact cross-method regression line.
                    //
                    // Raw and effective counts are both emitted.
                    // No ranking or success criterion is imposed on their
                    // relative values.
                    // --------------------------------------------------------
                    const bool controlledEffectiveFaceComparisonValid =
                        controlledCsgnEffectiveFaces
                                .valid_corridors ==
                            controlledSegmentCount &&
                        
                        controlledIdentityEffectiveFaces
                                .valid_corridors ==
                            controlledSegmentCount &&
                        
                        controlledFiriEffectiveFaces
                                .valid_corridors ==
                            controlledSegmentCount &&
                        
                        controlledRilsEffectiveFaces
                                .valid_corridors ==
                            controlledSegmentCount;
                        
                        
                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_EFFECTIVE_FACE_COMPARE "
                    
                        << "valid="
                        << controlledEffectiveFaceComparisonValid
                    
                        << " corridors="
                        << controlledSegmentCount
                    
                        << " csgn_raw="
                        << controlledCsgnEffectiveFaces
                               .raw_rows
                    
                        << " csgn_effective="
                        << controlledCsgnEffectiveFaces
                               .effective_faces
                    
                        << " identity_raw="
                        << controlledIdentityEffectiveFaces
                               .raw_rows
                    
                        << " identity_effective="
                        << controlledIdentityEffectiveFaces
                               .effective_faces
                    
                        << " firi_raw="
                        << controlledFiriEffectiveFaces
                               .raw_rows
                    
                        << " firi_effective="
                        << controlledFiriEffectiveFaces
                               .effective_faces
                    
                        << " rils_raw="
                        << controlledRilsEffectiveFaces
                               .raw_rows
                    
                        << " rils_effective="
                        << controlledRilsEffectiveFaces
                               .effective_faces);
                    
                    // ========================================================
                    // D1d-1: Common independent corridor safety measurement.
                    //
                    // Measurement only.
                    //
                    // Every controlled method is verified against:
                    //
                    //   1. the SAME global dilated-surface point cloud `pc`;
                    //
                    //   2. the SAME finite voxel-map bounds;
                    //
                    // using the SAME post-processing evaluator.
                    //
                    // This intentionally does NOT use:
                    //
                    //   Proposed constructor safety_verified,
                    //   FIRI unresolved constraints,
                    //   RILS generator success
                    //
                    // as the cross-method safety certificate.
                    //
                    // Those remain construction provenance only.
                    // ========================================================
                                        
                    struct ControlledSafetySummary
                    {
                        bool mapping_valid =
                            false;
                    
                        int valid_corridors =
                            0;
                    
                        int total_corridors =
                            0;
                    
                        int safe_corridors =
                            0;
                    
                        int obstacle_surface_safe_corridors =
                            0;
                    
                        int map_contained_corridors =
                            0;
                    
                        int obstacle_sample_count =
                            0;
                    
                        // Minimum signed exclusion margin over all valid
                        // corridors of this method.
                        double min_obstacle_exclusion_margin_m =
                            std::numeric_limits<double>::
                                infinity();
                    
                        // Largest penetration over all valid corridors.
                        double max_obstacle_penetration_m =
                            0.0;
                    
                        // Largest map-bound violation over all valid corridors.
                        //
                        // Negative means every corridor lies strictly inside
                        // the finite map.
                        double max_map_violation_m =
                            -std::numeric_limits<double>::
                                infinity();
                    };
                    
                    
                    // --------------------------------------------------------
                    // One common code path for all four methods.
                    // --------------------------------------------------------
                    auto evaluateControlledSafety =
                        [&](const std::string &methodName,
                            const std::vector<Eigen::MatrixX4d> &hPolys,
                            const bool mappingValid)
                            -> ControlledSafetySummary
                    {
                        ControlledSafetySummary summary;
                    
                        summary.mapping_valid =
                            mappingValid &&
                            static_cast<int>(
                                hPolys.size()) ==
                                controlledSegmentCount;
                            
                        summary.total_corridors =
                            controlledSegmentCount;
                            
                        summary.obstacle_sample_count =
                            static_cast<int>(
                                pc.size());
                            
                            
                        if (!summary.mapping_valid)
                        {
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_CORRIDOR_SAFETY_SUMMARY "
                            
                                << "method="
                                << methodName
                            
                                << " mapping_valid=0"
                            
                                << " valid=0"
                            
                                << " total="
                                << controlledSegmentCount
                            
                                << " safe=0"
                            
                                << " obstacle_surface_safe=0"
                            
                                << " map_contained=0"
                            
                                << " obstacle_samples="
                                << pc.size());
                            
                            return summary;
                        }
                    
                    
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            const auto metric =
                                gcopter_benchmark::
                                    evaluateCommonCorridorSafety(
                                        hPolys[
                                            corridorId],
                                        pc,
                                        voxelMap.getOrigin(),
                                        voxelMap.getCorner());
                                        
                                        
                            if (metric.valid)
                            {
                                ++summary.valid_corridors;
                            
                                if (metric.safe)
                                {
                                    ++summary.safe_corridors;
                                }
                            
                                if (metric.obstacle_surface_safe)
                                {
                                    ++summary
                                        .obstacle_surface_safe_corridors;
                                }
                            
                                if (metric.map_contained)
                                {
                                    ++summary
                                        .map_contained_corridors;
                                }
                            
                            
                                if (std::isfinite(
                                        metric
                                            .min_obstacle_exclusion_margin_m))
                                {
                                    summary
                                        .min_obstacle_exclusion_margin_m =
                                        std::min(
                                            summary
                                                .min_obstacle_exclusion_margin_m,
                                            metric
                                                .min_obstacle_exclusion_margin_m);
                                }
                            
                            
                                if (std::isfinite(
                                        metric
                                            .max_obstacle_penetration_m))
                                {
                                    summary
                                        .max_obstacle_penetration_m =
                                        std::max(
                                            summary
                                                .max_obstacle_penetration_m,
                                            metric
                                                .max_obstacle_penetration_m);
                                }
                            
                            
                                if (std::isfinite(
                                        metric
                                            .max_map_violation_m))
                                {
                                    summary
                                        .max_map_violation_m =
                                        std::max(
                                            summary
                                                .max_map_violation_m,
                                            metric
                                                .max_map_violation_m);
                                }
                            }
                        
                        
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_CORRIDOR_SAFETY "
                            
                                << "method="
                                << methodName
                            
                                << " corridor_id="
                                << corridorId
                            
                                << " valid="
                                << metric.valid
                            
                                << " safe="
                                << metric.safe
                            
                                << " obstacle_surface_safe="
                                << metric
                                       .obstacle_surface_safe
                            
                                << " map_contained="
                                << metric
                                       .map_contained
                            
                                << " obstacle_samples="
                                << metric
                                       .obstacle_sample_count
                            
                                << " worst_obstacle_index="
                                << metric
                                       .worst_obstacle_index
                            
                                << " min_obstacle_exclusion_margin_m="
                                << metric
                                       .min_obstacle_exclusion_margin_m
                            
                                << " max_obstacle_penetration_m="
                                << metric
                                       .max_obstacle_penetration_m
                            
                                << " max_map_violation_m="
                                << metric
                                       .max_map_violation_m);
                        }
                    
                    
                        if (!std::isfinite(
                                summary
                                    .min_obstacle_exclusion_margin_m))
                        {
                            summary
                                .min_obstacle_exclusion_margin_m =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        }
                    
                    
                        if (!std::isfinite(
                                summary
                                    .max_map_violation_m))
                        {
                            summary
                                .max_map_violation_m =
                                std::numeric_limits<double>::
                                    quiet_NaN();
                        }
                    
                    
                        ROS_INFO_STREAM(
                            "TF_CONTROLLED_CORRIDOR_SAFETY_SUMMARY "
                        
                            << "method="
                            << methodName
                        
                            << " mapping_valid="
                            << summary.mapping_valid
                        
                            << " valid="
                            << summary.valid_corridors
                        
                            << " total="
                            << summary.total_corridors
                        
                            << " safe="
                            << summary.safe_corridors
                        
                            << " obstacle_surface_safe="
                            << summary
                                   .obstacle_surface_safe_corridors
                        
                            << " map_contained="
                            << summary
                                   .map_contained_corridors
                        
                            << " obstacle_samples="
                            << summary
                                   .obstacle_sample_count
                        
                            << " min_obstacle_exclusion_margin_m="
                            << summary
                                   .min_obstacle_exclusion_margin_m
                        
                            << " max_obstacle_penetration_m="
                            << summary
                                   .max_obstacle_penetration_m
                        
                            << " max_map_violation_m="
                            << summary
                                   .max_map_violation_m);
                        
                        
                        return summary;
                    };
                    
                    
                    // ========================================================
                    // All four methods use exactly the same verifier.
                    //
                    // Fixed order keeps regression logs deterministic.
                    // ========================================================
                    
                    const auto controlledCsgnSafety =
                        evaluateControlledSafety(
                            "csgn",
                            controlledCsgnHPolys,
                            controlledPairMappingValid);
                        
                        
                    const auto controlledIdentitySafety =
                        evaluateControlledSafety(
                            "identity",
                            controlledIdentityHPolys,
                            controlledPairMappingValid);
                        
                        
                    const auto controlledFiriSafety =
                        evaluateControlledSafety(
                            "firi",
                            controlledFiriHPolys,
                            controlledFiriMappingValid);
                        
                        
                    const auto controlledRilsSafety =
                        evaluateControlledSafety(
                            "rils",
                            controlledRilsBuild.hpolys,
                            controlledRilsGeometryMappingValid);
                        
                        
                    // --------------------------------------------------------
                    // IMPORTANT:
                    //
                    // `valid` means the COMMON verifier ran successfully on
                    // every corridor.
                    //
                    // It does NOT silently require every baseline to be safe.
                    // Safety results remain experimental outcomes.
                    // --------------------------------------------------------
                    const bool controlledSafetyComparisonValid =
                        controlledCsgnSafety
                                .valid_corridors ==
                            controlledSegmentCount &&
                        
                        controlledIdentitySafety
                                .valid_corridors ==
                            controlledSegmentCount &&
                        
                        controlledFiriSafety
                                .valid_corridors ==
                            controlledSegmentCount &&
                        
                        controlledRilsSafety
                                .valid_corridors ==
                            controlledSegmentCount;
                        
                        
                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_CORRIDOR_SAFETY_COMPARE "
                    
                        << "valid="
                        << controlledSafetyComparisonValid
                    
                        << " corridors="
                        << controlledSegmentCount
                    
                        << " obstacle_samples="
                        << pc.size()
                    
                        << " csgn_safe="
                        << controlledCsgnSafety
                               .safe_corridors
                    
                        << " csgn_min_exclusion_m="
                        << controlledCsgnSafety
                               .min_obstacle_exclusion_margin_m
                    
                        << " csgn_max_penetration_m="
                        << controlledCsgnSafety
                               .max_obstacle_penetration_m
                    
                        << " identity_safe="
                        << controlledIdentitySafety
                               .safe_corridors
                    
                        << " identity_min_exclusion_m="
                        << controlledIdentitySafety
                               .min_obstacle_exclusion_margin_m
                    
                        << " identity_max_penetration_m="
                        << controlledIdentitySafety
                               .max_obstacle_penetration_m
                    
                        << " firi_safe="
                        << controlledFiriSafety
                               .safe_corridors
                    
                        << " firi_min_exclusion_m="
                        << controlledFiriSafety
                               .min_obstacle_exclusion_margin_m
                    
                        << " firi_max_penetration_m="
                        << controlledFiriSafety
                               .max_obstacle_penetration_m
                    
                        << " rils_safe="
                        << controlledRilsSafety
                               .safe_corridors
                    
                        << " rils_min_exclusion_m="
                        << controlledRilsSafety
                               .min_obstacle_exclusion_margin_m
                    
                        << " rils_max_penetration_m="
                        << controlledRilsSafety
                               .max_obstacle_penetration_m);

                    // ========================================================
                    // D1e-1: Final Controlled-E1 v3 corridor records.
                    //
                    // Exactly:
                    //
                    //     4 methods
                    //         x
                    //     original route segments
                    //
                    // using one common post-processing path.
                    //
                    // Final v3 methods:
                    //
                    //   proposed : CSGN + Active-Witness compact corridor
                    //   identity : Identity-metric Active-Witness ablation
                    //   firi     : Controlled standard FIRI
                    //   rils     : Controlled Liu/DecompUtil RILS
                    //
                    // IMPORTANT:
                    //
                    //   benchmark_corridors_v2.csv remains untouched.
                    //
                    //   benchmark_corridors_v3.csv is the final cross-method
                    //   Controlled-E1 geometry table.
                    //
                    // This entire block remains outside all frozen Proposed
                    // guide / CSGN / corridor / setup / optimize / hard timers.
                    // ========================================================


                    // --------------------------------------------------------
                    // Independent per-method one-segment -> one-corridor
                    // mapping validity.
                    //
                    // Do not couple Proposed validity to Identity validity here.
                    // The final CSV must preserve method-specific failures.
                    // --------------------------------------------------------
                    const bool controlledV3CsgnMappingValid =
                        controlledCsgnSuccess &&
                        static_cast<int>(
                            controlledCsgnHPolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledCsgnInfos.size()) ==
                            controlledSegmentCount;
                        
                        
                    const bool controlledV3IdentityMappingValid =
                        controlledIdentitySuccess &&
                        static_cast<int>(
                            controlledIdentityHPolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledIdentityInfos.size()) ==
                            controlledSegmentCount;
                        
                        
                    const bool controlledV3FiriMappingValid =
                        controlledFiriSuccess &&
                        static_cast<int>(
                            controlledFiriHPolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledFiriDiagnostics.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledFiriLocalObstacleCounts.size()) ==
                            controlledSegmentCount;
                        
                        
                    const bool controlledV3RilsMappingValid =
                        controlledRilsBuild.success &&
                        controlledRilsBuild.mapping_valid &&
                        static_cast<int>(
                            controlledRilsBuild.hpolys.size()) ==
                            controlledSegmentCount &&
                        static_cast<int>(
                            controlledRilsBuild.infos.size()) ==
                            controlledSegmentCount;
                        
                        
                    // ========================================================
                    // One frozen CSGN reference basis per ORIGINAL route
                    // segment.
                    //
                    // All four methods are measured against these SAME
                    // eigendirections.
                    //
                    // SelfAdjointEigenSolver returns ascending eigenvalues:
                    //
                    //   eig0 = hard / low deformation utility
                    //   eig1 = middle
                    //   eig2 = easy / high deformation utility
                    // ========================================================
                        
                    struct ControlledV3ReferenceBasis
                    {
                        bool valid =
                            false;
                    
                        Eigen::Vector3d eigenvalues =
                            Eigen::Vector3d::Constant(
                                std::numeric_limits<double>::
                                    quiet_NaN());
                            
                        Eigen::Matrix3d directions =
                            Eigen::Matrix3d::Identity();
                            
                        int source_piece_id =
                            -1;
                            
                        double mapping_distance =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    };


                    std::vector<
                        ControlledV3ReferenceBasis>
                        controlledV3ReferenceBases(
                            std::max(
                                0,
                                controlledSegmentCount));
                            
                            
                    for (int segmentId = 0;
                         segmentId <
                             controlledSegmentCount;
                         ++segmentId)
                    {
                        ControlledV3ReferenceBasis &basis =
                            controlledV3ReferenceBases[
                                segmentId];
                            
                            
                        if (segmentId >=
                                static_cast<int>(
                                    guideSegmentMetrics.size()) ||
                            !guideSegmentMetrics[
                                 segmentId]
                                 .valid ||
                            !guideSegmentMetrics[
                                 segmentId]
                                 .utility
                                 .allFinite())
                        {
                            continue;
                        }
                    
                    
                        Eigen::Matrix3d utility =
                            guideSegmentMetrics[
                                segmentId]
                                .utility;
                            
                            
                        utility =
                            0.5 *
                            (
                                utility +
                                utility.transpose());
                            
                            
                        Eigen::SelfAdjointEigenSolver<
                            Eigen::Matrix3d>
                            utilitySolver(
                                utility);
                            
                            
                        if (utilitySolver.info() !=
                                Eigen::Success ||
                            utilitySolver
                                    .eigenvalues()
                                    .minCoeff() <=
                                0.0)
                        {
                            continue;
                        }
                    
                    
                        basis.eigenvalues =
                            utilitySolver
                                .eigenvalues();
                    
                        basis.directions =
                            utilitySolver
                                .eigenvectors();
                    
                    
                        // Same deterministic eigenvector sign convention as
                        // all previous Controlled E1 measurements.
                        for (int directionId = 0;
                             directionId < 3;
                             ++directionId)
                        {
                            Eigen::Vector3d direction =
                                basis.directions.col(
                                    directionId);
                                
                            Eigen::Index pivotId =
                                0;
                                
                                
                            direction
                                .cwiseAbs()
                                .maxCoeff(
                                    &pivotId);
                                
                                
                            if (direction(
                                    pivotId) <
                                0.0)
                            {
                                direction =
                                    -direction;
                            }
                        
                        
                            basis.directions.col(
                                directionId) =
                                direction;
                        }
                    
                    
                        basis.source_piece_id =
                            guideSegmentMetrics[
                                segmentId]
                                .source_piece_id;
                            
                        basis.mapping_distance =
                            guideSegmentMetrics[
                                segmentId]
                                .mapping_distance;
                            
                        basis.valid =
                            true;
                    }


                    // ========================================================
                    // Final v3 row buffer.
                    //
                    // Even if one method fails mapping, we still create one row
                    // per intended original segment, with
                    //
                    //     geometry_mapping_valid = false
                    //
                    // and invalid common metrics.
                    //
                    // This prevents silent deletion of failed baselines from the
                    // future route-bank statistics.
                    // ========================================================

                    std::vector<
                        gcopter_benchmark::
                            BenchmarkControlledCorridorRecordV3>
                        benchmarkControlledCorridorRecordsV3;


                    benchmarkControlledCorridorRecordsV3.reserve(
                        4 *
                        std::max(
                            0,
                            controlledSegmentCount));
                        
                        
                    constexpr double
                        controlledV3GeometryToleranceM =
                            1.0e-8;
                        
                        
                    int controlledV3FaceAccountingMismatchCount =
                        0;
                        
                    int controlledV3EffectiveRawMismatchCount =
                        0;
                        
                        
                    // ========================================================
                    // Shared row constructor.
                    //
                    // `fillConstructionProvenance()` is the ONLY method-specific
                    // part.
                    //
                    // Everything geometrical below is measured by the same:
                    //
                    //   evaluateCommonCorridorSafety()
                    //   evaluateSegmentSeedRadius()
                    //   evaluateJunctionOverlapRadius()
                    //   evaluatePointDirectionalWidth()
                    //   evaluateHPolytopeVolume()
                    //   evaluateEffectiveFaces()
                    //
                    // kernels.
                    // ========================================================
                        
                    auto appendControlledV3Method =
                        [&](const std::string &methodName,
                            const std::string &variantName,
                            const std::string &constructionAlgorithm,
                            const std::string &constructionBasis,
                            const std::vector<Eigen::MatrixX4d> &hPolys,
                            const bool mappingValid,
                            const bool protectedRadiusPrescribed,
                            const auto &fillConstructionProvenance)
                    {
                        for (int corridorId = 0;
                             corridorId <
                                 controlledSegmentCount;
                             ++corridorId)
                        {
                            gcopter_benchmark::
                                BenchmarkControlledCorridorRecordV3
                                    record;
                        
                        
                            // ====================================================
                            // Identity / pairing provenance.
                            // ====================================================
                            record.case_id =
                                effectiveCaseId;
                        
                            record.route_fingerprint =
                                routeFingerprint;
                        
                            record.method =
                                methodName;
                        
                            record.variant =
                                variantName;
                        
                            record.repeat_id =
                                config.benchmarkRepeatId;
                        
                            record.timestamp_s =
                                benchmarkRunReady
                                    ? benchmarkRun.timestamp_s
                                    : ros::Time::now().toSec();
                        
                            record.corridor_id =
                                corridorId;
                        
                            record.source_segment_id =
                                corridorId;
                        
                            record.geometry_mapping_valid =
                                mappingValid;
                        
                            record.geometry_protocol =
                                "controlled_geometry";
                        
                        
                            // ====================================================
                            // Construction provenance.
                            // ====================================================
                            record.construction_algorithm =
                                constructionAlgorithm;
                        
                            record.construction_direction_basis =
                                constructionBasis;
                        
                            record.reference_direction_source =
                                "direct_minco_csgn";
                        
                            // Every method begins from the same global shared
                            // dilated surface cloud.
                            record.input_obstacle_count =
                                static_cast<int>(
                                    pc.size());
                                
                                
                            // ====================================================
                            // COMMON reference CSGN metric provenance exists
                            // independently of whether this method's construction
                            // succeeded.
                            // ====================================================
                            if (corridorId <
                                    static_cast<int>(
                                        controlledV3ReferenceBases.size()) &&
                                controlledV3ReferenceBases[
                                    corridorId]
                                    .valid)
                            {
                                const auto &basis =
                                    controlledV3ReferenceBases[
                                        corridorId];
                                    
                                    
                                record.reference_metric_valid =
                                    true;
                                    
                                record.reference_utility_eig0 =
                                    basis.eigenvalues(0);
                                    
                                record.reference_utility_eig1 =
                                    basis.eigenvalues(1);
                                    
                                record.reference_utility_eig2 =
                                    basis.eigenvalues(2);
                                    
                                record.reference_metric_source_piece_id =
                                    basis.source_piece_id;
                                    
                                record.reference_metric_mapping_distance =
                                    basis.mapping_distance;
                            }
                        
                        
                            // ====================================================
                            // If construction/mapping failed, preserve an explicit
                            // failure row rather than indexing incomplete vectors.
                            // ====================================================
                            if (mappingValid)
                            {
                                const Eigen::MatrixX4d &hPoly =
                                    hPolys[
                                        corridorId];
                                    
                                    
                                // ------------------------------------------------
                                // Actual stored H-row workload.
                                // ------------------------------------------------
                                record.raw_face_count =
                                    static_cast<int>(
                                        hPoly.rows());
                                    
                                    
                                // Method-specific construction provenance only.
                                fillConstructionProvenance(
                                    record,
                                    corridorId);
                                
                                
                                if (record.raw_domain_face_count +
                                        record.raw_obstacle_face_count !=
                                    record.raw_face_count)
                                {
                                    ++controlledV3FaceAccountingMismatchCount;
                                }
                            
                            
                                // =================================================
                                // COMMON independent safety.
                                // =================================================
                                const auto safetyMetric =
                                    gcopter_benchmark::
                                        evaluateCommonCorridorSafety(
                                            hPoly,
                                            pc,
                                            voxelMap.getOrigin(),
                                            voxelMap.getCorner());
                                        
                                        
                                record.common_safety_valid =
                                    safetyMetric.valid;
                                        
                                record.common_safe =
                                    safetyMetric.safe;
                                        
                                record.obstacle_surface_safe =
                                    safetyMetric
                                        .obstacle_surface_safe;
                                        
                                record.map_contained =
                                    safetyMetric
                                        .map_contained;
                                        
                                record.obstacle_sample_count =
                                    safetyMetric
                                        .obstacle_sample_count;
                                        
                                record.worst_obstacle_index =
                                    safetyMetric
                                        .worst_obstacle_index;
                                        
                                record.min_obstacle_exclusion_margin_m =
                                    safetyMetric
                                        .min_obstacle_exclusion_margin_m;
                                        
                                record.max_obstacle_penetration_m =
                                    safetyMetric
                                        .max_obstacle_penetration_m;
                                        
                                record.max_map_violation_m =
                                    safetyMetric
                                        .max_map_violation_m;
                                        
                                        
                                // =================================================
                                // COMMON seed radius.
                                // =================================================
                                const auto seedMetric =
                                    gcopter_benchmark::
                                        evaluateSegmentSeedRadius(
                                            hPoly,
                                            route[
                                                corridorId],
                                            route[
                                                corridorId + 1]);
                                            
                                            
                                record.seed_metric_valid =
                                    seedMetric.valid;
                                            
                                record.seed_radius_m =
                                    seedMetric.radius_m;
                                            
                                            
                                record.protected_radius_prescribed =
                                    protectedRadiusPrescribed;
                                            
                                            
                                if (protectedRadiusPrescribed)
                                {
                                    record.prescribed_protected_radius_m =
                                        protectedRadiusM;
                                
                                    record.prescribed_seed_satisfied =
                                        seedMetric.valid &&
                                        seedMetric.radius_m +
                                                controlledV3GeometryToleranceM >=
                                            protectedRadiusM;
                                }
                            
                            
                                // =================================================
                                // COMMON neighboring junction overlap.
                                //
                                // Last corridor has no successor:
                                //
                                //     junction_overlap_valid = false
                                //
                                // by definition.
                                // =================================================
                                gcopter_benchmark::
                                    CorridorOverlapMetric
                                        overlapMetric;
                            
                            
                                if (corridorId + 1 <
                                    controlledSegmentCount)
                                {
                                    overlapMetric =
                                        gcopter_benchmark::
                                            evaluateJunctionOverlapRadius(
                                                hPoly,
                                                hPolys[
                                                    corridorId + 1],
                                                route[
                                                    corridorId + 1]);
                                }
                            
                            
                                record.junction_overlap_valid =
                                    overlapMetric.valid;
                            
                                record.junction_overlap_radius_m =
                                    overlapMetric.radius_m;
                            
                            
                                if (protectedRadiusPrescribed &&
                                    corridorId + 1 <
                                        controlledSegmentCount)
                                {
                                    record.prescribed_overlap_satisfied =
                                        overlapMetric.valid &&
                                        overlapMetric.radius_m +
                                                controlledV3GeometryToleranceM >=
                                            protectedRadiusM;
                                }
                            
                            
                                // =================================================
                                // COMMON midpoint directional width.
                                // =================================================
                                if (record.reference_metric_valid)
                                {
                                    const auto &basis =
                                        controlledV3ReferenceBases[
                                            corridorId];
                                        
                                        
                                    const Eigen::Vector3d
                                        directionalReferencePoint =
                                            0.5 *
                                            (
                                                route[
                                                    corridorId] +
                                                route[
                                                    corridorId + 1]);
                                                
                                                
                                    const auto hardWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                hPoly,
                                                directionalReferencePoint,
                                                basis
                                                    .directions
                                                    .col(0));
                                            
                                            
                                    const auto middleWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                hPoly,
                                                directionalReferencePoint,
                                                basis
                                                    .directions
                                                    .col(1));
                                            
                                            
                                    const auto easyWidth =
                                        gcopter_benchmark::
                                            evaluatePointDirectionalWidth(
                                                hPoly,
                                                directionalReferencePoint,
                                                basis
                                                    .directions
                                                    .col(2));
                                            
                                            
                                    record.point_width_valid =
                                        hardWidth.valid &&
                                        middleWidth.valid &&
                                        easyWidth.valid &&
                                        hardWidth.reference_inside &&
                                        middleWidth.reference_inside &&
                                        easyWidth.reference_inside;
                                            
                                            
                                    if (record.point_width_valid)
                                    {
                                        record.point_reference_margin_m =
                                            std::min(
                                                hardWidth
                                                    .min_reference_margin_m,
                                                std::min(
                                                    middleWidth
                                                        .min_reference_margin_m,
                                                    easyWidth
                                                        .min_reference_margin_m));
                                                
                                                
                                        record.hard_positive_m =
                                            hardWidth.positive_m;
                                                
                                        record.hard_negative_m =
                                            hardWidth.negative_m;
                                                
                                        record.hard_width_m =
                                            hardWidth.width_m;
                                                
                                                
                                        record.middle_positive_m =
                                            middleWidth.positive_m;
                                                
                                        record.middle_negative_m =
                                            middleWidth.negative_m;
                                                
                                        record.middle_width_m =
                                            middleWidth.width_m;
                                                
                                                
                                        record.easy_positive_m =
                                            easyWidth.positive_m;
                                                
                                        record.easy_negative_m =
                                            easyWidth.negative_m;
                                                
                                        record.easy_width_m =
                                            easyWidth.width_m;
                                    }
                                }
                            
                            
                                // =================================================
                                // COMMON final H-polytope volume.
                                // =================================================
                                const auto volumeMetric =
                                    gcopter_benchmark::
                                        evaluateHPolytopeVolume(
                                            hPoly);
                                        
                                        
                                record.volume_valid =
                                    volumeMetric.valid;
                                        
                                        
                                if (volumeMetric.valid)
                                {
                                    record.volume_m3 =
                                        volumeMetric.volume_m3;
                                
                                    record.volume_vertex_count =
                                        volumeMetric.vertex_count;
                                
                                    record.volume_triangle_count =
                                        volumeMetric.triangle_count;
                                
                                    record.volume_max_vertex_violation_m =
                                        volumeMetric
                                            .max_vertex_violation_m;
                                }
                            
                            
                                // =================================================
                                // COMMON LP-based effective-face count.
                                // =================================================
                                const auto effectiveFaceMetric =
                                    gcopter_benchmark::
                                        evaluateEffectiveFaces(
                                            hPoly);
                                        
                                        
                                record.effective_face_valid =
                                    effectiveFaceMetric.valid;
                                        
                                        
                                if (effectiveFaceMetric.valid)
                                {
                                    record.unique_plane_groups =
                                        effectiveFaceMetric
                                            .unique_plane_groups;
                                
                                    record.duplicate_rows =
                                        effectiveFaceMetric
                                            .duplicate_rows;
                                
                                    record.effective_face_count =
                                        effectiveFaceMetric
                                            .effective_faces;
                                
                                    record.redundant_plane_groups =
                                        effectiveFaceMetric
                                            .redundant_plane_groups;
                                
                                
                                    if (effectiveFaceMetric.raw_rows !=
                                        record.raw_face_count)
                                    {
                                        ++controlledV3EffectiveRawMismatchCount;
                                    }
                                }
                            }
                        
                        
                            benchmarkControlledCorridorRecordsV3
                                .push_back(
                                    record);
                                
                                
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_E1_V3_ROW "
                            
                                << "method="
                                << record.method
                            
                                << " variant="
                                << record.variant
                            
                                << " corridor_id="
                                << record.corridor_id
                            
                                << " mapping_valid="
                                << record.geometry_mapping_valid
                            
                                << " safety_valid="
                                << record.common_safety_valid
                            
                                << " safe="
                                << record.common_safe
                            
                                << " seed_valid="
                                << record.seed_metric_valid
                            
                                << " seed_radius_m="
                                << record.seed_radius_m
                            
                                << " protected_prescribed="
                                << record.protected_radius_prescribed
                            
                                << " prescribed_seed_satisfied="
                                << record.prescribed_seed_satisfied
                            
                                << " overlap_valid="
                                << record.junction_overlap_valid
                            
                                << " overlap_radius_m="
                                << record.junction_overlap_radius_m
                            
                                << " prescribed_overlap_satisfied="
                                << record.prescribed_overlap_satisfied
                            
                                << " width_valid="
                                << record.point_width_valid
                            
                                << " hard_width_m="
                                << record.hard_width_m
                            
                                << " mid_width_m="
                                << record.middle_width_m
                            
                                << " easy_width_m="
                                << record.easy_width_m
                            
                                << " volume_valid="
                                << record.volume_valid
                            
                                << " volume_m3="
                                << record.volume_m3
                            
                                << " raw_faces="
                                << record.raw_face_count
                            
                                << " effective_valid="
                                << record.effective_face_valid
                            
                                << " effective_faces="
                                << record.effective_face_count);
                        }
                    };


                    // ========================================================
                    // Proposed / CSGN Active-Witness.
                    // ========================================================

                    appendControlledV3Method(
                        "proposed",
                        "csgn_active_controlled",
                        "active_witness_compact",
                        "csgn_eigenbasis",
                        controlledCsgnHPolys,
                        controlledV3CsgnMappingValid,
                        true,
                        [&](gcopter_benchmark::
                                BenchmarkControlledCorridorRecordV3 &record,
                            const int corridorId)
                        {
                            const auto &info =
                                controlledCsgnInfos[
                                    corridorId];
                                
                                
                            record.local_obstacle_count =
                                info.local_obstacle_count;
                                
                            record.raw_domain_face_count =
                                info.domain_face_count;
                                
                            record.raw_obstacle_face_count =
                                info.selected_obstacle_face_count;
                                
                            record.aw_candidate_count =
                                info.candidate_count;
                                
                            record.aw_active_rounds =
                                info.active_witness_rounds;
                                
                            record.aw_witness_distance_tests =
                                info.witness_distance_tests;
                                
                            record.aw_obstacle_face_tests =
                                info.obstacle_face_tests;
                        });
                    
                    
                    // ========================================================
                    // Identity Active-Witness ablation.
                    // ========================================================
                    
                    appendControlledV3Method(
                        "identity",
                        "identity_active_controlled",
                        "active_witness_compact",
                        "world_xyz_identity",
                        controlledIdentityHPolys,
                        controlledV3IdentityMappingValid,
                        true,
                        [&](gcopter_benchmark::
                                BenchmarkControlledCorridorRecordV3 &record,
                            const int corridorId)
                        {
                            const auto &info =
                                controlledIdentityInfos[
                                    corridorId];
                                
                                
                            record.local_obstacle_count =
                                info.local_obstacle_count;
                                
                            record.raw_domain_face_count =
                                info.domain_face_count;
                                
                            record.raw_obstacle_face_count =
                                info.selected_obstacle_face_count;
                                
                            record.aw_candidate_count =
                                info.candidate_count;
                                
                            record.aw_active_rounds =
                                info.active_witness_rounds;
                                
                            record.aw_witness_distance_tests =
                                info.witness_distance_tests;
                                
                            record.aw_obstacle_face_tests =
                                info.obstacle_face_tests;
                        });
                    
                    
                    // ========================================================
                    // Controlled standard FIRI.
                    //
                    // Active-Witness counters deliberately remain -1.
                    // ========================================================
                    
                    appendControlledV3Method(
                        "firi",
                        "standard_firi_controlled",
                        "standard_firi_reference",
                        "standard_firi_euclidean",
                        controlledFiriHPolys,
                        controlledV3FiriMappingValid,
                        false,
                        [&](gcopter_benchmark::
                                BenchmarkControlledCorridorRecordV3 &record,
                            const int corridorId)
                        {
                            const auto &diagnostics =
                                controlledFiriDiagnostics[
                                    corridorId];
                                
                                
                            record.local_obstacle_count =
                                controlledFiriLocalObstacleCounts[
                                    corridorId];
                                
                            record.raw_obstacle_face_count =
                                diagnostics
                                    .obstacle_face_count;
                                
                            record.raw_domain_face_count =
                                std::max(
                                    0,
                                    record.raw_face_count -
                                        record.raw_obstacle_face_count);
                        });
                    
                    
                    // ========================================================
                    // Controlled Liu / DecompUtil RILS.
                    //
                    // Active-Witness counters deliberately remain -1.
                    //
                    // raw domain faces:
                    //
                    //     local RILS bbox
                    //         +
                    //     appended finite GCOPTER map bounds.
                    // ========================================================
                    
                    appendControlledV3Method(
                        "rils",
                        "liu_rils_controlled",
                        "liu_decompros_rils_reference",
                        "decompros_ellipsoid",
                        controlledRilsBuild.hpolys,
                        controlledV3RilsMappingValid,
                        false,
                        [&](gcopter_benchmark::
                                BenchmarkControlledCorridorRecordV3 &record,
                            const int corridorId)
                        {
                            const auto &info =
                                controlledRilsBuild.infos[
                                    corridorId];
                                
                                
                            record.local_obstacle_count =
                                info.local_obstacle_count;
                                
                            record.raw_obstacle_face_count =
                                info.obstacle_faces;
                                
                            record.raw_domain_face_count =
                                info.local_domain_faces +
                                info.map_domain_faces;
                        });
                    
                    
                    // ========================================================
                    // Final v3 structural / measurement audit.
                    //
                    // Do NOT require `common_safe` here:
                    //
                    // safety is an experimental result, not a condition for
                    // deciding whether the data row itself is valid.
                    //
                    // Likewise, prescribed protection is only applicable to
                    // Proposed and Identity.
                    // ========================================================
                    
                    const int controlledV3ExpectedRows =
                        4 *
                        std::max(
                            0,
                            controlledSegmentCount);
                        
                        
                    const int controlledV3ExpectedOverlapRows =
                        4 *
                        std::max(
                            0,
                            controlledSegmentCount - 1);
                        
                        
                    int controlledV3MappingRows =
                        0;
                        
                    int controlledV3SafetyValidRows =
                        0;
                        
                    int controlledV3SafeRows =
                        0;
                        
                    int controlledV3SeedValidRows =
                        0;
                        
                    int controlledV3OverlapValidRows =
                        0;
                        
                    int controlledV3ReferenceMetricValidRows =
                        0;
                        
                    int controlledV3PointWidthValidRows =
                        0;
                        
                    int controlledV3VolumeValidRows =
                        0;
                        
                    int controlledV3EffectiveFaceValidRows =
                        0;
                        
                    int controlledV3ProtectedPrescribedRows =
                        0;
                        
                    int controlledV3PrescribedSeedSatisfiedRows =
                        0;
                        
                    int controlledV3PrescribedOverlapSatisfiedRows =
                        0;
                        
                        
                    int controlledV3ProposedRows =
                        0;
                        
                    int controlledV3IdentityRows =
                        0;
                        
                    int controlledV3FiriRows =
                        0;
                        
                    int controlledV3RilsRows =
                        0;
                        
                        
                    int controlledV3ProposedRawFaces =
                        0;
                        
                    int controlledV3IdentityRawFaces =
                        0;
                        
                    int controlledV3FiriRawFaces =
                        0;
                        
                    int controlledV3RilsRawFaces =
                        0;
                        
                        
                    int controlledV3ProposedEffectiveFaces =
                        0;
                        
                    int controlledV3IdentityEffectiveFaces =
                        0;
                        
                    int controlledV3FiriEffectiveFaces =
                        0;
                        
                    int controlledV3RilsEffectiveFaces =
                        0;
                        
                        
                    for (const auto &record :
                         benchmarkControlledCorridorRecordsV3)
                    {
                        controlledV3MappingRows +=
                            record.geometry_mapping_valid
                                ? 1
                                : 0;
                    
                        controlledV3SafetyValidRows +=
                            record.common_safety_valid
                                ? 1
                                : 0;
                    
                        controlledV3SafeRows +=
                            record.common_safe
                                ? 1
                                : 0;
                    
                        controlledV3SeedValidRows +=
                            record.seed_metric_valid
                                ? 1
                                : 0;
                    
                        controlledV3OverlapValidRows +=
                            record.junction_overlap_valid
                                ? 1
                                : 0;
                    
                        controlledV3ReferenceMetricValidRows +=
                            record.reference_metric_valid
                                ? 1
                                : 0;
                    
                        controlledV3PointWidthValidRows +=
                            record.point_width_valid
                                ? 1
                                : 0;
                    
                        controlledV3VolumeValidRows +=
                            record.volume_valid
                                ? 1
                                : 0;
                    
                        controlledV3EffectiveFaceValidRows +=
                            record.effective_face_valid
                                ? 1
                                : 0;
                    
                        controlledV3ProtectedPrescribedRows +=
                            record.protected_radius_prescribed
                                ? 1
                                : 0;
                    
                        controlledV3PrescribedSeedSatisfiedRows +=
                            record.prescribed_seed_satisfied
                                ? 1
                                : 0;
                    
                        controlledV3PrescribedOverlapSatisfiedRows +=
                            record.prescribed_overlap_satisfied
                                ? 1
                                : 0;
                    
                    
                        if (record.method ==
                            "proposed")
                        {
                            ++controlledV3ProposedRows;
                        
                            controlledV3ProposedRawFaces +=
                                record.raw_face_count;
                        
                            controlledV3ProposedEffectiveFaces +=
                                record.effective_face_count;
                        }
                        else if (record.method ==
                                 "identity")
                        {
                            ++controlledV3IdentityRows;
                        
                            controlledV3IdentityRawFaces +=
                                record.raw_face_count;
                        
                            controlledV3IdentityEffectiveFaces +=
                                record.effective_face_count;
                        }
                        else if (record.method ==
                                 "firi")
                        {
                            ++controlledV3FiriRows;
                        
                            controlledV3FiriRawFaces +=
                                record.raw_face_count;
                        
                            controlledV3FiriEffectiveFaces +=
                                record.effective_face_count;
                        }
                        else if (record.method ==
                                 "rils")
                        {
                            ++controlledV3RilsRows;
                        
                            controlledV3RilsRawFaces +=
                                record.raw_face_count;
                        
                            controlledV3RilsEffectiveFaces +=
                                record.effective_face_count;
                        }
                    }


                    // --------------------------------------------------------
                    // Schema completeness, NOT experimental method success.
                    //
                    // This is intentionally strict for the fixed dev regression.
                    // On a route-bank case where one baseline construction fails,
                    // the v3 rows are still written, but schema_valid becomes 0.
                    // --------------------------------------------------------

                    const bool controlledV3SchemaValid =
                        static_cast<int>(
                            benchmarkControlledCorridorRecordsV3.size()) ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3ProposedRows ==
                            controlledSegmentCount &&
                        
                        controlledV3IdentityRows ==
                            controlledSegmentCount &&
                        
                        controlledV3FiriRows ==
                            controlledSegmentCount &&
                        
                        controlledV3RilsRows ==
                            controlledSegmentCount &&
                        
                        controlledV3MappingRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3SafetyValidRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3SeedValidRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3OverlapValidRows ==
                            controlledV3ExpectedOverlapRows &&
                        
                        controlledV3ReferenceMetricValidRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3PointWidthValidRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3VolumeValidRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3EffectiveFaceValidRows ==
                            controlledV3ExpectedRows &&
                        
                        controlledV3FaceAccountingMismatchCount ==
                            0 &&
                        
                        controlledV3EffectiveRawMismatchCount ==
                            0;
                        
                        
                    // ========================================================
                    // Write final Controlled-E1 v3 table.
                    //
                    // Keep v2 logging immediately below unchanged.
                    // ========================================================
                        
                    bool benchmarkControlledCorridorV3LogSuccess =
                        true;
                        
                        
                    if (benchmarkRunReady)
                    {
                        benchmarkControlledCorridorV3LogSuccess =
                            benchmarkCorridorLogger
                                .logControlledCorridorsV3(
                                    benchmarkControlledCorridorRecordsV3);
                                
                                
                        if (!benchmarkControlledCorridorV3LogSuccess)
                        {
                            ROS_ERROR(
                                "Failed to append "
                                "benchmark_corridors_v3.csv.");
                        }
                    }


                    ROS_INFO_STREAM(
                        "TF_BENCHMARK_CORRIDORS_V3 "
                    
                        << "schema_valid="
                        << controlledV3SchemaValid
                    
                        << " rows="
                        << benchmarkControlledCorridorRecordsV3
                               .size()
                    
                        << " expected_rows="
                        << controlledV3ExpectedRows
                    
                        << " mapping_rows="
                        << controlledV3MappingRows
                    
                        << " safety_valid_rows="
                        << controlledV3SafetyValidRows
                    
                        << " safe_rows="
                        << controlledV3SafeRows
                    
                        << " seed_valid_rows="
                        << controlledV3SeedValidRows
                    
                        << " overlap_valid_rows="
                        << controlledV3OverlapValidRows
                    
                        << " expected_overlap_rows="
                        << controlledV3ExpectedOverlapRows
                    
                        << " reference_metric_valid_rows="
                        << controlledV3ReferenceMetricValidRows
                    
                        << " point_width_valid_rows="
                        << controlledV3PointWidthValidRows
                    
                        << " volume_valid_rows="
                        << controlledV3VolumeValidRows
                    
                        << " effective_face_valid_rows="
                        << controlledV3EffectiveFaceValidRows
                    
                        << " protected_prescribed_rows="
                        << controlledV3ProtectedPrescribedRows
                    
                        << " prescribed_seed_satisfied_rows="
                        << controlledV3PrescribedSeedSatisfiedRows
                    
                        << " prescribed_overlap_satisfied_rows="
                        << controlledV3PrescribedOverlapSatisfiedRows
                    
                        << " face_accounting_mismatches="
                        << controlledV3FaceAccountingMismatchCount
                    
                        << " effective_raw_mismatches="
                        << controlledV3EffectiveRawMismatchCount
                    
                        << " proposed_raw="
                        << controlledV3ProposedRawFaces
                    
                        << " proposed_effective="
                        << controlledV3ProposedEffectiveFaces
                    
                        << " identity_raw="
                        << controlledV3IdentityRawFaces
                    
                        << " identity_effective="
                        << controlledV3IdentityEffectiveFaces
                    
                        << " firi_raw="
                        << controlledV3FiriRawFaces
                    
                        << " firi_effective="
                        << controlledV3FiriEffectiveFaces
                    
                        << " rils_raw="
                        << controlledV3RilsRawFaces
                    
                        << " rils_effective="
                        << controlledV3RilsEffectiveFaces
                    
                        << " log_success="
                        << benchmarkControlledCorridorV3LogSuccess
                    
                        << " file=benchmark_corridors_v3.csv");

                    // ========================================================
                    // D2a-0 / E2-0:
                    // Four-method COMMON soft-GCOPTER backend replay.
                    //
                    // Experimental control:
                    //
                    //   same route
                    //   same initial / final PVA states
                    //   same GCOPTER backend implementation
                    //   same physical parameters
                    //   same dynamic limits
                    //   same penalty weights
                    //   same quadrature resolution
                    //   same optimizer stopping tolerance
                    //
                    // ONLY the controlled corridor set changes.
                    //
                    // IMPORTANT:
                    //
                    // This is a post-hoc E2 diagnostic block.
                    // Its setup_ms / optimize_ms are NOT part of the frozen E4
                    // Proposed timing protocol.
                    //
                    // E2 compares the SOFT common-backend trajectories.
                    // Exact-Hard remains a separate E5 experiment.
                    // ========================================================

                    struct ControlledE2BackendEvaluation
                    {
                        bool mapping_valid =
                            false;
                    
                        bool backend_attempted =
                            false;
                    
                        BackendAbResult backend;
                    
                        bool trajectory_rebuild_ready =
                            false;
                    
                        double rebuilt_smoothness_energy =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    
                        double rebuild_duration_delta_s =
                            std::numeric_limits<double>::
                                quiet_NaN();
                    
                        gcopter_benchmark::
                            FinalTrajectoryMetrics
                                trajectory_metrics;
                    };


                    // --------------------------------------------------------
                    // One common E2 execution path.
                    //
                    // runBackendAb() performs the SAME GCOPTER setup/optimize
                    // for every corridor method.
                    //
                    // We then reconstruct the exact optimized MINCO state from
                    // the returned internal waypoints and piece times and feed it
                    // to the already-frozen common trajectory evaluator.
                    // --------------------------------------------------------
                    auto evaluateControlledE2Backend =
                        [&](const std::string &methodName,
                            const std::vector<Eigen::MatrixX4d> &hPolys,
                            const bool mappingValid)
                            -> ControlledE2BackendEvaluation
                    {
                        ControlledE2BackendEvaluation result;
                    
                        result.mapping_valid =
                            mappingValid &&
                            static_cast<int>(
                                hPolys.size()) ==
                                controlledSegmentCount;
                            
                            
                        if (!result.mapping_valid)
                        {
                            ROS_INFO_STREAM(
                                "TF_CONTROLLED_E2_BACKEND "
                            
                                << "method="
                                << methodName
                            
                                << " mapping_valid=0"
                            
                                << " backend_attempted=0"
                            
                                << " setup_success=0"
                            
                                << " optimize_success=0"
                            
                                << " optimized_state_ready=0"
                            
                                << " rebuild_ready=0"
                            
                                << " metrics_valid=0");
                            
                            return result;
                        }
                    
                    
                        result.backend_attempted =
                            true;
                    
                    
                        // ====================================================
                        // SAME backend call for every method.
                        // ====================================================
                        result.backend =
                            runBackendAb(
                                hPolys);
                            
                            
                        // ====================================================
                        // Reconstruct exactly the optimized soft MINCO state.
                        //
                        // No additional optimization occurs here.
                        // ====================================================
                        if (result.backend
                                .optimized_state_ready)
                        {
                            const int pieceCount =
                                static_cast<int>(
                                    result.backend
                                        .optimized_times
                                        .size());
                                
                                
                            const bool stateValid =
                                pieceCount > 0 &&
                                
                                result.backend
                                        .optimized_points
                                        .rows() ==
                                    3 &&
                                
                                result.backend
                                        .optimized_points
                                        .cols() ==
                                    pieceCount - 1 &&
                                
                                result.backend
                                        .optimized_points
                                        .allFinite() &&
                                
                                result.backend
                                        .optimized_times
                                        .allFinite() &&
                                
                                (
                                    result.backend
                                        .optimized_times
                                        .array() >
                                    0.0)
                                    .all();
                                
                                
                            if (stateValid)
                            {
                                minco::MINCO_S3NU
                                    rebuiltMinco;
                            
                            
                                rebuiltMinco.setConditions(
                                    iniState,
                                    finState,
                                    pieceCount);
                                
                                
                                rebuiltMinco.setParameters(
                                    result.backend
                                        .optimized_points,
                                    result.backend
                                        .optimized_times);
                                
                                
                                Trajectory<5>
                                    rebuiltTrajectory;
                                
                                
                                rebuiltMinco.getTrajectory(
                                    rebuiltTrajectory);
                                
                                
                                rebuiltMinco.getEnergy(
                                    result
                                        .rebuilt_smoothness_energy);
                                
                                
                                result.trajectory_rebuild_ready =
                                    rebuiltTrajectory
                                            .getPieceNum() ==
                                        pieceCount &&
                                
                                    std::isfinite(
                                        result
                                            .rebuilt_smoothness_energy);
                                    
                                    
                                if (result
                                        .trajectory_rebuild_ready)
                                {
                                    result.trajectory_metrics =
                                        gcopter_benchmark::
                                            evaluateFinalTrajectoryMetrics(
                                                rebuiltTrajectory,
                                                result
                                                    .rebuilt_smoothness_energy,
                                                config.weightT,
                                                config.vehicleMass,
                                                config.gravAcc,
                                                config.horizDrag,
                                                config.vertDrag,
                                                config.parasDrag,
                                                config.speedEps,
                                                1.0e-3);
                                            
                                            
                                    if (result
                                            .trajectory_metrics
                                            .valid &&
                                        std::isfinite(
                                            result.backend
                                                .trajectory_duration))
                                    {
                                        result
                                            .rebuild_duration_delta_s =
                                            result
                                                .trajectory_metrics
                                                .duration_s -
                                            result.backend
                                                .trajectory_duration;
                                    }
                                }
                            }
                        }
                    
                    
                        // ====================================================
                        // One complete E2 diagnostic line.
                        //
                        // Precision semantics:
                        //
                        // vmax / amax:
                        //     polynomial root-based exact extrema
                        //
                        // length:
                        //     deterministic Simpson quadrature, dt <= 1 ms
                        //
                        // body-rate / tilt / thrust:
                        //     sampled, dt <= 1 ms
                        //
                        // exact_contained:
                        //     exact polynomial continuous-time SFC certificate
                        //     for the SOFT GCOPTER trajectory.
                        // ====================================================
                        ROS_INFO_STREAM(
                            "TF_CONTROLLED_E2_BACKEND "
                        
                            << "method="
                            << methodName
                        
                            << " mapping_valid="
                            << result.mapping_valid
                        
                            << " backend_attempted="
                            << result.backend_attempted
                        
                            << " setup_success="
                            << result.backend
                                   .setup_success
                        
                            << " optimize_success="
                            << result.backend
                                   .optimize_success
                        
                            << " optimized_state_ready="
                            << result.backend
                                   .optimized_state_ready
                        
                            << " rebuild_ready="
                            << result
                                   .trajectory_rebuild_ready
                        
                            << " metrics_valid="
                            << result
                                   .trajectory_metrics
                                   .valid
                        
                            << " corridors="
                            << result.backend
                                   .corridor_count
                        
                            << " raw_faces="
                            << result.backend
                                   .total_faces
                        
                            << " pieces="
                            << result.backend
                                   .trajectory_pieces
                        
                            << " setup_ms="
                            << result.backend
                                   .setup_ms
                        
                            << " optimize_ms="
                            << result.backend
                                   .optimize_ms
                        
                            << " optimizer_cost="
                            << result.backend
                                   .final_cost
                        
                            << " duration_s="
                            << result
                                   .trajectory_metrics
                                   .duration_s
                        
                            << " length_m="
                            << result
                                   .trajectory_metrics
                                   .length_m
                        
                            << " smoothness_energy="
                            << result
                                   .trajectory_metrics
                                   .smoothness_energy
                        
                            << " time_cost="
                            << result
                                   .trajectory_metrics
                                   .time_cost
                        
                            << " j_kin="
                            << result
                                   .trajectory_metrics
                                   .j_kin
                        
                            << " max_vel_mps="
                            << result
                                   .trajectory_metrics
                                   .max_velocity_mps
                        
                            << " max_acc_mps2="
                            << result
                                   .trajectory_metrics
                                   .max_acceleration_mps2
                        
                            << " max_body_rate_radps="
                            << result
                                   .trajectory_metrics
                                   .max_body_rate_radps
                        
                            << " max_tilt_rad="
                            << result
                                   .trajectory_metrics
                                   .max_tilt_rad
                        
                            << " min_thrust_N="
                            << result
                                   .trajectory_metrics
                                   .min_thrust_n
                        
                            << " max_thrust_N="
                            << result
                                   .trajectory_metrics
                                   .max_thrust_n
                        
                            << " rebuild_duration_delta_s="
                            << result
                                   .rebuild_duration_delta_s
                        
                            << " soft_exact_mapping_valid="
                            << result.backend
                                   .exact_mapping_valid
                        
                            << " soft_exact_cert_valid="
                            << result.backend
                                   .exact_certificate_valid
                        
                            << " soft_exact_contained="
                            << result.backend
                                   .exact_contained
                        
                            << " soft_exact_violation_m="
                            << result.backend
                                   .exact_max_violation_m
                        
                            << " soft_exact_min_margin_m="
                            << result.backend
                                   .exact_min_margin_m);
                        
                        
                        return result;
                    };


                    // ========================================================
                    // Fixed E2 execution order.
                    //
                    // Do not change this order across experiments.
                    // ========================================================

                    const auto controlledE2Proposed =
                        evaluateControlledE2Backend(
                            "proposed",
                            controlledCsgnHPolys,
                            controlledV3CsgnMappingValid);
                        
                        
                    const auto controlledE2Identity =
                        evaluateControlledE2Backend(
                            "identity",
                            controlledIdentityHPolys,
                            controlledV3IdentityMappingValid);
                        
                        
                    const auto controlledE2Firi =
                        evaluateControlledE2Backend(
                            "firi",
                            controlledFiriHPolys,
                            controlledV3FiriMappingValid);
                        
                        
                    const auto controlledE2Rils =
                        evaluateControlledE2Backend(
                            "rils",
                            controlledRilsBuild.hpolys,
                            controlledV3RilsMappingValid);
                        
                        
                    // ========================================================
                    // Proposed replay / determinism cross-check.
                    //
                    // The Controlled Proposed H-polytopes have already been
                    // independently checked against the native Proposed set.
                    //
                    // Now verify that sending those corridors through the SAME
                    // backend reproduces the original Proposed soft optimizer
                    // state.
                    //
                    // This test reports RAW deltas.  It does not alter success.
                    // ========================================================
                        
                    bool controlledE2ProposedReplayComparable =
                        controlledV3CsgnMappingValid &&
                        controlledCsgnNativeMatchValid &&
                        
                        activeGuideBackendResult
                            .setup_success &&
                        activeGuideBackendResult
                            .optimize_success &&
                        activeGuideBackendResult
                            .optimized_state_ready &&
                        
                        controlledE2Proposed
                            .backend
                            .setup_success &&
                        controlledE2Proposed
                            .backend
                            .optimize_success &&
                        controlledE2Proposed
                            .backend
                            .optimized_state_ready;
                        
                        
                    double controlledE2ProposedPointDelta =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                    double controlledE2ProposedTimeDelta =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                    double controlledE2ProposedCostDelta =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                    double controlledE2ProposedDurationDelta =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                    double controlledE2ProposedSoftJkinDelta =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                    double controlledE2ProposedSoftLengthDeltaM =
                        std::numeric_limits<double>::
                            quiet_NaN();
                        
                        
                    if (controlledE2ProposedReplayComparable)
                    {
                        const bool pointShapeMatch =
                            activeGuideBackendResult
                                    .optimized_points
                                    .rows() ==
                                controlledE2Proposed
                                    .backend
                                    .optimized_points
                                    .rows() &&
                    
                            activeGuideBackendResult
                                    .optimized_points
                                    .cols() ==
                                controlledE2Proposed
                                    .backend
                                    .optimized_points
                                    .cols();
                    
                    
                        const bool timeShapeMatch =
                            activeGuideBackendResult
                                    .optimized_times
                                    .size() ==
                                controlledE2Proposed
                                    .backend
                                    .optimized_times
                                    .size();
                    
                    
                        if (!pointShapeMatch ||
                            !timeShapeMatch)
                        {
                            controlledE2ProposedReplayComparable =
                                false;
                        }
                        else
                        {
                            controlledE2ProposedPointDelta =
                                (
                                    activeGuideBackendResult
                                        .optimized_points -
                                    controlledE2Proposed
                                        .backend
                                        .optimized_points)
                                    .cwiseAbs()
                                    .maxCoeff();
                                
                                
                            controlledE2ProposedTimeDelta =
                                (
                                    activeGuideBackendResult
                                        .optimized_times -
                                    controlledE2Proposed
                                        .backend
                                        .optimized_times)
                                    .cwiseAbs()
                                    .maxCoeff();
                                
                                
                            controlledE2ProposedCostDelta =
                                std::abs(
                                    activeGuideBackendResult
                                        .final_cost -
                                    controlledE2Proposed
                                        .backend
                                        .final_cost);
                                
                                
                            controlledE2ProposedDurationDelta =
                                std::abs(
                                    activeGuideBackendResult
                                        .trajectory_duration -
                                    controlledE2Proposed
                                        .backend
                                        .trajectory_duration);
                                
                                
                            if (softTrajectoryEvaluation.valid &&
                                controlledE2Proposed
                                    .trajectory_metrics
                                    .valid)
                            {
                                controlledE2ProposedSoftJkinDelta =
                                    std::abs(
                                        softTrajectoryEvaluation
                                            .j_kin -
                                        controlledE2Proposed
                                            .trajectory_metrics
                                            .j_kin);
                                    
                                    
                                controlledE2ProposedSoftLengthDeltaM =
                                    std::abs(
                                        softTrajectoryEvaluation
                                            .length_m -
                                        controlledE2Proposed
                                            .trajectory_metrics
                                            .length_m);
                            }
                        }
                    }


                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_E2_PROPOSED_REPLAY "
                    
                        << "comparable="
                        << controlledE2ProposedReplayComparable
                    
                        << " corridor_native_match="
                        << controlledCsgnNativeMatchValid
                    
                        << " max_point_delta_m="
                        << controlledE2ProposedPointDelta
                    
                        << " max_time_delta_s="
                        << controlledE2ProposedTimeDelta
                    
                        << " optimizer_cost_delta="
                        << controlledE2ProposedCostDelta
                    
                        << " duration_delta_s="
                        << controlledE2ProposedDurationDelta
                    
                        << " soft_j_kin_delta="
                        << controlledE2ProposedSoftJkinDelta
                    
                        << " soft_length_delta_m="
                        << controlledE2ProposedSoftLengthDeltaM);
                    
                    
                    // ========================================================
                    // Compact four-method E2 execution audit.
                    //
                    // `valid` means all four methods produced a reconstructed,
                    // measurable common-backend soft trajectory.
                    //
                    // Do NOT require continuous-time exact containment here:
                    // that is a measured outcome, not the definition of backend
                    // optimization success.
                    // ========================================================
                    
                    const int controlledE2MetricValidCount =
                        (controlledE2Proposed
                             .trajectory_metrics
                             .valid
                             ? 1
                             : 0) +
                        
                        (controlledE2Identity
                             .trajectory_metrics
                             .valid
                             ? 1
                             : 0) +
                        
                        (controlledE2Firi
                             .trajectory_metrics
                             .valid
                             ? 1
                             : 0) +
                        
                        (controlledE2Rils
                             .trajectory_metrics
                             .valid
                             ? 1
                             : 0);
                        
                        
                    const int controlledE2SetupSuccessCount =
                        (controlledE2Proposed
                             .backend
                             .setup_success
                             ? 1
                             : 0) +
                        
                        (controlledE2Identity
                             .backend
                             .setup_success
                             ? 1
                             : 0) +
                        
                        (controlledE2Firi
                             .backend
                             .setup_success
                             ? 1
                             : 0) +
                        
                        (controlledE2Rils
                             .backend
                             .setup_success
                             ? 1
                             : 0);
                        
                        
                    const int controlledE2OptimizeSuccessCount =
                        (controlledE2Proposed
                             .backend
                             .optimize_success
                             ? 1
                             : 0) +
                        
                        (controlledE2Identity
                             .backend
                             .optimize_success
                             ? 1
                             : 0) +
                        
                        (controlledE2Firi
                             .backend
                             .optimize_success
                             ? 1
                             : 0) +
                        
                        (controlledE2Rils
                             .backend
                             .optimize_success
                             ? 1
                             : 0);
                        
                        
                    const int controlledE2SoftExactContainedCount =
                        (controlledE2Proposed
                             .backend
                             .exact_contained
                             ? 1
                             : 0) +
                        
                        (controlledE2Identity
                             .backend
                             .exact_contained
                             ? 1
                             : 0) +
                        
                        (controlledE2Firi
                             .backend
                             .exact_contained
                             ? 1
                             : 0) +
                        
                        (controlledE2Rils
                             .backend
                             .exact_contained
                             ? 1
                             : 0);
                        
                        
                    const bool controlledE2ExecutionValid =
                        controlledE2SetupSuccessCount ==
                            4 &&
                        
                        controlledE2OptimizeSuccessCount ==
                            4 &&
                        
                        controlledE2MetricValidCount ==
                            4;
                        
                        
                    ROS_INFO_STREAM(
                        "TF_CONTROLLED_E2_COMPARE "
                    
                        << "valid="
                        << controlledE2ExecutionValid
                    
                        << " setup_success="
                        << controlledE2SetupSuccessCount
                    
                        << " optimize_success="
                        << controlledE2OptimizeSuccessCount
                    
                        << " metrics_valid="
                        << controlledE2MetricValidCount
                    
                        << " soft_exact_contained="
                        << controlledE2SoftExactContainedCount
                    
                        << " proposed_j_kin="
                        << controlledE2Proposed
                               .trajectory_metrics
                               .j_kin
                    
                        << " identity_j_kin="
                        << controlledE2Identity
                               .trajectory_metrics
                               .j_kin
                    
                        << " firi_j_kin="
                        << controlledE2Firi
                               .trajectory_metrics
                               .j_kin
                    
                        << " rils_j_kin="
                        << controlledE2Rils
                               .trajectory_metrics
                               .j_kin
                    
                        << " proposed_length_m="
                        << controlledE2Proposed
                               .trajectory_metrics
                               .length_m
                    
                        << " identity_length_m="
                        << controlledE2Identity
                               .trajectory_metrics
                               .length_m
                    
                        << " firi_length_m="
                        << controlledE2Firi
                               .trajectory_metrics
                               .length_m
                    
                        << " rils_length_m="
                        << controlledE2Rils
                               .trajectory_metrics
                               .length_m);

                    // ========================================================
                    // D2a-1:
                    // Final four-method Controlled E2 CSV.
                    //
                    // Exactly one row per method and replay case.
                    //
                    // IMPORTANT:
                    //
                    // This serializes the already-computed D2a-0 results.
                    // It does NOT run another optimizer and does NOT modify any
                    // corridor or trajectory.
                    // ========================================================
                                        
                    std::vector<
                        gcopter_benchmark::
                            BenchmarkControlledE2Record>
                        benchmarkControlledE2Records;
                                        
                                        
                    benchmarkControlledE2Records.reserve(
                        4);
                    
                    
                    const double controlledE2TimestampS =
                        ros::Time::now().toSec();
                    
                    
                    auto appendControlledE2Record =
                        [&](const std::string &methodName,
                            const std::string &variantName,
                            const ControlledE2BackendEvaluation &evaluation)
                    {
                        gcopter_benchmark::
                            BenchmarkControlledE2Record
                                record;
                    
                    
                        record.case_id =
                            effectiveCaseId;
                    
                        record.route_fingerprint =
                            routeFingerprint;
                    
                        record.method =
                            methodName;
                    
                        record.variant =
                            variantName;
                    
                        record.repeat_id =
                            config.benchmarkRepeatId;
                    
                        record.timestamp_s =
                            controlledE2TimestampS;
                    
                    
                        record.mapping_valid =
                            evaluation.mapping_valid;
                    
                        record.backend_attempted =
                            evaluation.backend_attempted;
                    
                        record.setup_success =
                            evaluation.backend
                                .setup_success;
                    
                        record.optimize_success =
                            evaluation.backend
                                .optimize_success;
                    
                        record.optimized_state_ready =
                            evaluation.backend
                                .optimized_state_ready;
                    
                        record.trajectory_rebuild_ready =
                            evaluation
                                .trajectory_rebuild_ready;
                    
                        record.trajectory_metrics_valid =
                            evaluation
                                .trajectory_metrics
                                .valid;
                    
                    
                        record.corridor_count =
                            evaluation.backend
                                .corridor_count;
                    
                        record.raw_face_count =
                            evaluation.backend
                                .total_faces;
                    
                        record.trajectory_piece_count =
                            evaluation.backend
                                .trajectory_pieces;
                    
                    
                        record.setup_ms =
                            evaluation.backend
                                .setup_ms;
                    
                        record.optimize_ms =
                            evaluation.backend
                                .optimize_ms;
                    
                        record.optimizer_cost =
                            evaluation.backend
                                .final_cost;
                    
                    
                        record.duration_s =
                            evaluation
                                .trajectory_metrics
                                .duration_s;
                    
                        record.length_m =
                            evaluation
                                .trajectory_metrics
                                .length_m;
                    
                        record.smoothness_energy =
                            evaluation
                                .trajectory_metrics
                                .smoothness_energy;
                    
                        record.time_cost =
                            evaluation
                                .trajectory_metrics
                                .time_cost;
                    
                        record.j_kin =
                            evaluation
                                .trajectory_metrics
                                .j_kin;
                    
                        record.max_velocity_mps =
                            evaluation
                                .trajectory_metrics
                                .max_velocity_mps;
                    
                        record.max_acceleration_mps2 =
                            evaluation
                                .trajectory_metrics
                                .max_acceleration_mps2;
                    
                        record.max_body_rate_radps =
                            evaluation
                                .trajectory_metrics
                                .max_body_rate_radps;
                    
                        record.max_tilt_rad =
                            evaluation
                                .trajectory_metrics
                                .max_tilt_rad;
                    
                        record.min_thrust_n =
                            evaluation
                                .trajectory_metrics
                                .min_thrust_n;
                    
                        record.max_thrust_n =
                            evaluation
                                .trajectory_metrics
                                .max_thrust_n;
                    
                    
                        record.rebuild_duration_delta_s =
                            evaluation
                                .rebuild_duration_delta_s;
                    
                    
                        record.soft_exact_mapping_valid =
                            evaluation.backend
                                .exact_mapping_valid;
                    
                        record.soft_exact_certificate_valid =
                            evaluation.backend
                                .exact_certificate_valid;
                    
                        record.soft_exact_contained =
                            evaluation.backend
                                .exact_contained;
                    
                        record.soft_exact_max_violation_m =
                            evaluation.backend
                                .exact_max_violation_m;
                    
                        record.soft_exact_min_margin_m =
                            evaluation.backend
                                .exact_min_margin_m;
                    
                        record.soft_exact_certificate_ms =
                            evaluation.backend
                                .exact_certificate_ms;
                    
                    
                        benchmarkControlledE2Records
                            .push_back(
                                record);
                    };
                    
                    
                    appendControlledE2Record(
                        "proposed",
                        "csgn_active_controlled",
                        controlledE2Proposed);
                    
                    
                    appendControlledE2Record(
                        "identity",
                        "identity_active_controlled",
                        controlledE2Identity);
                    
                    
                    appendControlledE2Record(
                        "firi",
                        "standard_firi_controlled",
                        controlledE2Firi);
                    
                    
                    appendControlledE2Record(
                        "rils",
                        "liu_rils_controlled",
                        controlledE2Rils);
                    
                    
                    // --------------------------------------------------------
                    // Structural validity only.
                    //
                    // Experimental failures remain rows and must NOT disappear
                    // from route-bank statistics.
                    // --------------------------------------------------------
                    const bool controlledE2CsvSchemaValid =
                        benchmarkControlledE2Records.size() ==
                            4;
                    
                    
                    bool benchmarkControlledE2LogSuccess =
                        true;
                    
                    
                    if (benchmarkRunReady)
                    {
                        benchmarkControlledE2LogSuccess =
                            benchmarkRunLogger
                                .logControlledE2(
                                    benchmarkControlledE2Records);
                                
                                
                        if (!benchmarkControlledE2LogSuccess)
                        {
                            ROS_ERROR(
                                "Failed to append "
                                "benchmark_e2_v1.csv.");
                        }
                    }
                    
                    
                    ROS_INFO_STREAM(
                        "TF_BENCHMARK_E2 "
                    
                        << "schema_valid="
                        << controlledE2CsvSchemaValid
                    
                        << " rows="
                        << benchmarkControlledE2Records
                               .size()
                    
                        << " setup_success="
                        << controlledE2SetupSuccessCount
                    
                        << " optimize_success="
                        << controlledE2OptimizeSuccessCount
                    
                        << " metrics_valid="
                        << controlledE2MetricValidCount
                    
                        << " soft_exact_contained="
                        << controlledE2SoftExactContainedCount
                    
                        << " proposed_replay_comparable="
                        << controlledE2ProposedReplayComparable
                    
                        << " log_success="
                        << benchmarkControlledE2LogSuccess
                    
                        << " file=benchmark_e2_v1.csv");

                    bool benchmarkCorridorLogSuccess =
                        true;  

                    if (benchmarkRunReady)
                    {
                        benchmarkCorridorLogSuccess =
                            benchmarkCorridorLogger
                                .logCorridors(
                                    benchmarkControlledCorridorRecords);
                                
                        if (!benchmarkCorridorLogSuccess)
                        {
                            ROS_ERROR(
                                "Failed to append "
                                "benchmark_corridors_v2.csv.");
                        }
                    }
                    
                    ROS_INFO_STREAM(
                        "TF_BENCHMARK_CORRIDORS "
                        << "rows="
                        << benchmarkControlledCorridorRecords
                               .size()
                    
                        << " expected_rows="
                        << 2 *
                               std::max(
                                   0,
                                   controlledSegmentCount)
                            
                        << " mapping_valid="
                        << controlledPairMappingValid
                            
                        << " log_success="
                        << benchmarkCorridorLogSuccess
                            
                        << " backend_final_success="
                        << proposedFinalSuccess
                            
                        << " protocol=controlled_geometry"
                            
                        << " file=benchmark_corridors_v2.csv");
                            
                            
                    // ========================================================
                    // E2/E5 success-only trajectory finalization.
                    //
                    // E1 geometry has already been measured/logged above,
                    // regardless of backend success.
                    // ========================================================
                    if (proposedFinalSuccess)
                    {
                    
                        // All success-path trajectory measurements are now available.
                        // Emit exactly one structured benchmark row.
                        if (benchmarkRunReady)
                        {
                        // ====================================================
                        // FINAL trajectory = exact-hard trajectory.
                        // ====================================================
                        benchmarkRun.trajectory_piece_count =
                            finalTrajectoryEvaluation
                                .piece_count;
                    
                        benchmarkRun.trajectory_duration_s =
                            finalTrajectoryEvaluation
                                .duration_s;
                    
                        benchmarkRun.trajectory_metrics_valid =
                            finalTrajectoryEvaluation
                                .valid;
                    
                        benchmarkRun.trajectory_length_m =
                            finalTrajectoryEvaluation
                                .length_m;
                    
                        benchmarkRun.smoothness_energy =
                            finalTrajectoryEvaluation
                                .smoothness_energy;
                    
                        benchmarkRun.time_cost =
                            finalTrajectoryEvaluation
                                .time_cost;
                    
                        benchmarkRun.j_kin =
                            finalTrajectoryEvaluation
                                .j_kin;
                    
                        benchmarkRun.max_velocity_mps =
                            finalTrajectoryEvaluation
                                .max_velocity_mps;
                    
                        benchmarkRun.max_acceleration_mps2 =
                            finalTrajectoryEvaluation
                                .max_acceleration_mps2;
                    
                        benchmarkRun.max_body_rate_radps =
                            finalTrajectoryEvaluation
                                .max_body_rate_radps;
                    
                        benchmarkRun.max_tilt_rad =
                            finalTrajectoryEvaluation
                                .max_tilt_rad;
                    
                        benchmarkRun.min_thrust_n =
                            finalTrajectoryEvaluation
                                .min_thrust_n;
                    
                        benchmarkRun.max_thrust_n =
                            finalTrajectoryEvaluation
                                .max_thrust_n;
                    
                    
                        // ====================================================
                        // SOFT source trajectory = same GCOPTER optimum
                        // consumed by the hard projection.
                        // ====================================================
                        benchmarkRun.soft_trajectory_metrics_valid =
                            softTrajectoryEvaluation
                                .valid;
                    
                        benchmarkRun.soft_trajectory_duration_s =
                            softTrajectoryEvaluation
                                .duration_s;
                    
                        benchmarkRun.soft_trajectory_length_m =
                            softTrajectoryEvaluation
                                .length_m;
                    
                        benchmarkRun.soft_smoothness_energy =
                            softTrajectoryEvaluation
                                .smoothness_energy;
                    
                        benchmarkRun.soft_time_cost =
                            softTrajectoryEvaluation
                                .time_cost;
                    
                        benchmarkRun.soft_j_kin =
                            softTrajectoryEvaluation
                                .j_kin;
                    
                        benchmarkRun.soft_max_velocity_mps =
                            softTrajectoryEvaluation
                                .max_velocity_mps;
                    
                        benchmarkRun.soft_max_acceleration_mps2 =
                            softTrajectoryEvaluation
                                .max_acceleration_mps2;
                    
                        benchmarkRun.soft_max_body_rate_radps =
                            softTrajectoryEvaluation
                                .max_body_rate_radps;
                    
                        benchmarkRun.soft_max_tilt_rad =
                            softTrajectoryEvaluation
                                .max_tilt_rad;
                    
                        benchmarkRun.soft_min_thrust_n =
                            softTrajectoryEvaluation
                                .min_thrust_n;
                    
                        benchmarkRun.soft_max_thrust_n =
                            softTrajectoryEvaluation
                                .max_thrust_n;
                    
                    
                        // ====================================================
                        // Single-backend provenance.
                        // ====================================================
                        benchmarkRun.soft_hard_comparison_valid =
                            softHardComparisonValid;
                    
                        benchmarkRun.soft_rebuild_energy_delta =
                            softEnergyReferenceDelta;
                    
                        benchmarkRun.soft_rebuild_duration_delta_s =
                            softDurationReferenceDelta;
                    }

                    emitBenchmarkRun();
                        
                    visualizer.visualizePolytope(
                        activeGuideHPolys);

                    ROS_INFO_STREAM(
                        "TF_BENCHMARK_FINAL_TRAJ "
                        << "source=exact_hard_projection"
                        << " corridor_source=active_witness"
                        << " corridors="
                        << activeGuideHPolys.size()
                        << " faces="
                        << activeGuideTotalFaces
                        << " pieces="
                        << traj.getPieceNum()
                        << " duration="
                        << traj.getTotalDuration());

                    record.requested_method =
                        config.benchmarkMethod;

                    record.method =
                        config.benchmarkMethod;

                    record.corridor_generation_ms =
                        activeGuideMs;

                    record.corridor_count =
                        static_cast<int>(
                            activeGuideHPolys.size());

                    record.total_faces =
                        activeGuideTotalFaces;

                    record.mean_faces =
                        activeGuideHPolys.empty()
                            ? 0.0
                            : static_cast<double>(
                                  activeGuideTotalFaces) /
                                  static_cast<double>(
                                      activeGuideHPolys.size());

                    record.optimizer_setup_ms =
                        activeGuideBackendResult.setup_ms;

                    record.optimizer_ms =
                        activeGuideBackendResult.optimize_ms;

                    record.final_cost =
                        activeGuideBackendResult.final_cost;

                    record.corridor_constrained_piece_count =
                        activeGuideBackendResult
                            .constrained_pieces;

                    record.corridor_penalty_cost_initial =
                        activeGuideBackendResult
                            .corridor_penalty_initial;

                    record.corridor_penalty_cost_final =
                        activeGuideBackendResult
                            .corridor_penalty_final;

                    record.max_corridor_violation_initial_m =
                        activeGuideBackendResult
                            .max_corridor_violation_initial;

                    record.max_corridor_violation_final_m =
                        activeGuideBackendResult
                            .max_corridor_violation_final;

                    record.trajectory_piece_count =
                        finalTrajectoryEvaluation
                            .piece_count;
                                                    
                    record.trajectory_duration_s =
                        finalTrajectoryEvaluation
                            .duration_s;
                                                    
                    record.trajectory_length_m =
                        finalTrajectoryEvaluation
                            .length_m;

                    trajStamp =
                        ros::Time::now().toSec();

                    visualizer.visualize(
                        traj,
                        route);

                    finishRecord(
                        "success",
                        true);

                    return;
                }

                emitBenchmarkRun();

                record.requested_method =
                    config.benchmarkMethod;

                record.method =
                    config.benchmarkMethod;

                record.corridor_generation_ms =
                    activeGuideMs;

                record.corridor_count =
                    static_cast<int>(
                        activeGuideHPolys.size());

                record.total_faces =
                    activeGuideTotalFaces;

                record.optimizer_setup_ms =
                    activeGuideBackendResult.setup_ms;

                record.optimizer_ms =
                    activeGuideBackendResult.optimize_ms;

                record.final_cost =
                    activeGuideBackendResult.final_cost;

                if (!guideMetricSuccess ||
                    !guideSegmentMetricsReady)
                {
                    finishRecord(
                        "proposed_metric_failure",
                        false);
                }
                else if (!activeGuideSuccess)
                {
                    finishRecord(
                        "proposed_corridor_failure",
                        false);
                }
                else if (!activeGuideBackendResult
                              .setup_success)
                {
                    finishRecord(
                        "proposed_setup_failure",
                        false);
                }
                else if (!activeGuideBackendResult
                              .optimize_success)
                {
                    finishRecord(
                        "proposed_optimizer_failure",
                        false);
                }
                else
                {
                    finishRecord(
                        "proposed_exact_closure_failure",
                        false);
                }

                return;
            }

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
                visualizer.visualizePolytope(
                    hPolys);

                gcopter::GCOPTER_PolytopeSFC
                    gcopter;

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
                if (legacyDebugMode)
                {
                    ROS_INFO_STREAM(
                        "TF_GN_DEBUG_GATE "
                        << "experiment_tag="
                        << config.experimentTag
                        << " final_cost="
                        << record.final_cost
                        << " finite_cost="
                        << std::isfinite(
                               record.final_cost));
                }
                if (runProposedCore &&
                    std::isfinite(
                        record.final_cost))
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

                        int mincoNativeProjectionAttempts =
                            0;

                        int mincoNativeProjectionSuccesses =
                            0;

                        int mincoNativeProjectionIterations =
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

                            mincoNativeProjectionAttempts +=
                                info.projection_fallback_attempt_count;

                            mincoNativeProjectionSuccesses +=
                                info.projection_fallback_success_count;

                            mincoNativeProjectionIterations +=
                                info.projection_fallback_iteration_count;

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

                            << " projection_attempts="
                            << mincoNativeProjectionAttempts

                            << " projection_successes="
                            << mincoNativeProjectionSuccesses

                            << " projection_iterations="
                            << mincoNativeProjectionIterations

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


                        std::vector<Eigen::MatrixX4d>
                            guideCompactHPolys;

                        sfc_gen::TrajectoryRelevantCompactInfos
                            guideCompactInfos;

                        bool guideCompactSuccess =
                            false;

                        double guideCompactMs =
                            0.0;

                        if (legacyDebugMode &&
                            guideSegmentMetricsReady)
                        {
                            const auto guideCompactStarted =
                                std::chrono::steady_clock::now();
                        
                            guideCompactSuccess =
                                sfc_gen::
                                    trajectoryRelevantCompactCover(
                                        route,
                                        pc,
                                        voxelMap.getOrigin(),
                                        voxelMap.getCorner(),
                                        std::numeric_limits<double>::
                                            infinity(),
                                        guideCompactOptions,
                                        guideCompactHPolys,
                                        guideCompactInfos,
                                        &guideSegmentMetrics);
                                    
                            guideCompactMs =
                                std::chrono::duration<
                                    double,
                                    std::milli>(
                                        std::chrono::
                                            steady_clock::now() -
                                        guideCompactStarted)
                                    .count();
                        }

                        int guideCompactTotalFaces =
                            0;

                        int guideCompactDomainFaces =
                            0;

                        int guideCompactObstacleFaces =
                            0;

                        int guideCompactCandidates =
                            0;

                        int guideCompactGreedyFaces =
                            0;

                        int guideCompactRedundancyRemoved =
                            0;

                        std::int64_t guideCompactFaceTests =
                            0;

                        int guideCompactMetricValidCount =
                            0;

                        int guideCompactAnisotropicCount =
                            0;

                        int guideCompactSafetyCount =
                            0;

                        int guideCompactSeedOverlapCount =
                            0;

                        double guideCompactWeightedDamageSum =
                            0.0;

                        for (const auto &info :
                             guideCompactInfos)
                        {
                            guideCompactTotalFaces +=
                                info.total_face_count;

                            guideCompactDomainFaces +=
                                info.domain_face_count;

                            guideCompactObstacleFaces +=
                                info.selected_obstacle_face_count;

                            guideCompactCandidates +=
                                info.candidate_count;

                            guideCompactGreedyFaces +=
                                info.greedy_obstacle_face_count;

                            guideCompactRedundancyRemoved +=
                                info.redundancy_removed;

                            guideCompactFaceTests +=
                                info.obstacle_face_tests;

                            guideCompactMetricValidCount +=
                                info.metric_valid
                                    ? 1
                                    : 0;

                            guideCompactAnisotropicCount +=
                                info.anisotropic_domain
                                    ? 1
                                    : 0;

                            guideCompactSafetyCount +=
                                info.safety_verified
                                    ? 1
                                    : 0;

                            guideCompactSeedOverlapCount +=
                                info.overlap_guaranteed
                                    ? 1
                                    : 0;

                            guideCompactWeightedDamageSum +=
                                static_cast<double>(
                                    info.selected_obstacle_face_count) *
                                info.mean_metric_damage;
                        }

                        const double guideCompactMeanPsi =
                            guideCompactObstacleFaces > 0
                                ? guideCompactWeightedDamageSum /
                                      static_cast<double>(
                                          guideCompactObstacleFaces)
                                : 0.0;

                        // Explicitly verify actual adjacency after cover/shortcut.
                        int guideAdjacentOverlapValidCount =
                            0;

                        for (int corridorId = 1;
                             corridorId <
                                 static_cast<int>(
                                     guideCompactHPolys.size());
                             ++corridorId)
                        {
                            if (geo_utils::overlap(
                                    guideCompactHPolys[
                                        corridorId - 1],
                                    guideCompactHPolys[
                                        corridorId],
                                    0.01))
                            {
                                ++guideAdjacentOverlapValidCount;
                            }
                        }

                        const int guideAdjacentOverlapCount =
                            std::max(
                                0,
                                static_cast<int>(
                                    guideCompactHPolys.size()) -
                                    1);


                    if (legacyDebugMode)
                    {
                        ROS_INFO_STREAM(
                            "TF_GUIDE_COMPACT_AB "
                            << "success="
                            << guideCompactSuccess

                            << " metric_cardinality_valid="
                            << guideMetricCardinalityValid

                            << " route_segments="
                            << guideRawSegmentCount

                            << " guide_pieces="
                            << routeMincoGuide.getPieceNum()

                            << " guide_metrics="
                            << guideMetrics.size()

                            << " valid_metrics="
                            << guideSegmentMetricValidCount

                            << " corridors="
                            << guideCompactHPolys.size()

                            << " faces="
                            << guideCompactTotalFaces

                            << " domain_faces="
                            << guideCompactDomainFaces

                            << " obs_faces="
                            << guideCompactObstacleFaces

                            << " candidates="
                            << guideCompactCandidates

                            << " greedy_faces="
                            << guideCompactGreedyFaces

                            << " redundancy_removed="
                            << guideCompactRedundancyRemoved

                            << " mean_psi="
                            << guideCompactMeanPsi

                            << " metric_valid="
                            << guideCompactMetricValidCount
                            << "/"
                            << guideCompactInfos.size()

                            << " anisotropic="
                            << guideCompactAnisotropicCount
                            << "/"
                            << guideCompactInfos.size()

                            << " safety="
                            << guideCompactSafetyCount
                            << "/"
                            << guideCompactInfos.size()

                            << " seed_overlap="
                            << guideCompactSeedOverlapCount
                            << "/"
                            << guideCompactInfos.size()

                            << " adjacent_overlap="
                            << guideAdjacentOverlapValidCount
                            << "/"
                            << guideAdjacentOverlapCount

                            << " generation_ms="
                            << guideCompactMs);

                        ROS_INFO_STREAM(
                            "TF_GUIDE_ACTIVE_AB "
                            << "batch_success="
                            << guideCompactSuccess

                            << " active_success="
                            << activeGuideSuccess

                            << " batch_faces="
                            << guideCompactTotalFaces

                            << " active_faces="
                            << activeGuideTotalFaces

                            << " batch_obs_faces="
                            << guideCompactObstacleFaces

                            << " active_obs_faces="
                            << activeGuideObstacleFaces

                            << " batch_candidates="
                            << guideCompactCandidates

                            << " active_candidates="
                            << activeGuideCandidates

                            << " batch_domain_faces="
                            << guideCompactDomainFaces

                            << " active_domain_faces="
                            << activeGuideDomainFaces

                            << " batch_redundancy_removed="
                            << guideCompactRedundancyRemoved

                            << " active_redundancy_removed="
                            << activeGuideRedundancyRemoved

                            << " active_rounds="
                            << activeGuideRounds

                            << " batch_face_tests="
                            << guideCompactFaceTests

                            << " active_witness_tests="
                            << activeGuideWitnessTests

                            << " active_face_tests="
                            << activeGuideFaceTests

                            << " batch_ms="
                            << guideCompactMs

                            << " active_ms="
                            << activeGuideMs

                            << " active_safety="
                            << activeGuideSafetyCount
                            << "/"
                            << activeGuideInfos.size()

                            << " active_overlap="
                            << activeGuideAdjacentOverlapValidCount
                            << "/"
                            << activeGuideAdjacentOverlapCount);
                    }
                        BackendAbResult
                            guideCompactBackendResult;

                        if (legacyDebugMode &&
                            guideCompactSuccess)
                        {
                            guideCompactBackendResult =
                                runBackendAb(
                                    guideCompactHPolys);
                        }

                        BackendAbResult
                            activeGuidePenalty10Result;

                        if (legacyDebugMode &&
                            activeGuideSuccess)
                        {
                            activeGuidePenalty10Result =
                                runBackendAb(
                                    activeGuideHPolys,
                                    10.0);
                        }

                        bool warmContinuationSetupSuccess =
                            false;

                        bool warmContinuationBaseSuccess =
                            false;

                        bool warmContinuationSuccess =
                            false;

                        bool warmContinuationExactValid =
                            false;

                        bool warmContinuationExactContained =
                            false;

                        double warmContinuationBaseMs =
                            0.0;

                        double warmContinuationMs =
                            0.0;

                        double warmContinuationExactViolation =
                            std::numeric_limits<double>::
                                infinity();

                        double warmContinuationFinalCost =
                            std::numeric_limits<double>::
                                infinity();

                        Trajectory<5>
                            warmContinuationTrajectory;

                        if (legacyDebugMode && activeGuideSuccess)
                        {
                            gcopter::GCOPTER_PolytopeSFC
                                warmOptimizer;

                            warmContinuationSetupSuccess =
                                warmOptimizer.setup(
                                    config.weightT,
                                    iniState,
                                    finState,
                                    activeGuideHPolys,
                                    INFINITY,
                                    config.smoothingEps,
                                    quadratureRes,
                                    magnitudeBounds,
                                    penaltyWeights,
                                    physicalParams);

                            if (warmContinuationSetupSuccess)
                            {
                                Trajectory<5>
                                    baseTrajectory;

                                const auto baseStarted =
                                    std::chrono::
                                        steady_clock::now();

                                const double baseCost =
                                    warmOptimizer.optimize(
                                        baseTrajectory,
                                        config.relCostTol);

                                warmContinuationBaseMs =
                                    std::chrono::duration<
                                        double,
                                        std::milli>(
                                            std::chrono::
                                                steady_clock::now() -
                                            baseStarted)
                                        .count();

                                warmContinuationBaseSuccess =
                                    std::isfinite(
                                        baseCost) &&
                                    baseTrajectory.getPieceNum() ==
                                        static_cast<int>(
                                            activeGuideHPolys.size());

                                if (warmContinuationBaseSuccess)
                                {
                                    const auto continuationStarted =
                                        std::chrono::
                                            steady_clock::now();

                                    warmContinuationFinalCost =
                                        warmOptimizer
                                            .continueOptimizeWithCorridorPenaltyScale(
                                                warmContinuationTrajectory,
                                                config.relCostTol,
                                                10.0);

                                    warmContinuationMs =
                                        std::chrono::duration<
                                            double,
                                            std::milli>(
                                                std::chrono::
                                                    steady_clock::now() -
                                                continuationStarted)
                                            .count();

                                    warmContinuationSuccess =
                                        std::isfinite(
                                            warmContinuationFinalCost) &&
                                        warmContinuationTrajectory
                                                .getPieceNum() ==
                                            static_cast<int>(
                                                activeGuideHPolys.size());

                                    if (warmContinuationSuccess)
                                    {
                                        warmContinuationExactValid =
                                            true;

                                        warmContinuationExactContained =
                                            true;

                                        warmContinuationExactViolation =
                                            -std::numeric_limits<double>::
                                                infinity();

                                        for (int pieceId = 0;
                                             pieceId <
                                                 warmContinuationTrajectory
                                                     .getPieceNum();
                                             ++pieceId)
                                        {
                                            const auto cert =
                                                traj_relevant::
                                                    certifyMincoPieceInPolytope(
                                                        warmContinuationTrajectory[
                                                            pieceId],
                                                        activeGuideHPolys[
                                                            pieceId],
                                                        1.0e-6,
                                                        1.0e-10,
                                                        1.0e-12);

                                            if (!cert.valid)
                                            {
                                                warmContinuationExactValid =
                                                    false;

                                                warmContinuationExactContained =
                                                    false;

                                                break;
                                            }

                                            warmContinuationExactContained =
                                                warmContinuationExactContained &&
                                                cert.contained;

                                            warmContinuationExactViolation =
                                                std::max(
                                                    warmContinuationExactViolation,
                                                    cert.max_signed_violation_m);
                                        }
                                    }
                                }
                            }
                        }

                        const double guideProposedComponentMs =
                            routeMincoGuideBuildMs +
                            guideMetricMs +
                            guideCompactMs +
                            guideCompactBackendResult.setup_ms +
                            guideCompactBackendResult.optimize_ms;

                        const double activeGuideProposedComponentMs =
                            routeMincoGuideBuildMs +
                            guideMetricMs +
                            activeGuideMs +
                            activeGuideBackendResult.setup_ms +
                            activeGuideBackendResult.optimize_ms;
                        if (legacyDebugMode)
                        {
                        ROS_INFO_STREAM(
                            "TF_GUIDE_COMPACT_BACKEND "
                            << "corridor_success="
                            << guideCompactSuccess

                            << " setup_success="
                            << guideCompactBackendResult
                                   .setup_success

                            << " opt_success="
                            << guideCompactBackendResult
                                   .optimize_success

                            << " corridors="
                            << guideCompactBackendResult
                                   .corridor_count

                            << " faces="
                            << guideCompactBackendResult
                                   .total_faces

                            << " traj_pieces="
                            << guideCompactBackendResult
                                   .trajectory_pieces

                            << " constrained_pieces="
                            << guideCompactBackendResult
                                   .constrained_pieces

                            << " final_cost="
                            << guideCompactBackendResult
                                   .final_cost

                            << " duration="
                            << guideCompactBackendResult
                                   .trajectory_duration

                            << " setup_ms="
                            << guideCompactBackendResult
                                   .setup_ms

                            << " opt_ms="
                            << guideCompactBackendResult
                                   .optimize_ms

                            << " penalty_initial="
                            << guideCompactBackendResult
                                   .corridor_penalty_initial

                            << " penalty_final="
                            << guideCompactBackendResult
                                   .corridor_penalty_final

                            << " violation_initial="
                            << guideCompactBackendResult
                                   .max_corridor_violation_initial

                            << " violation_final="
                            << guideCompactBackendResult
                                   .max_corridor_violation_final

                            << " slack_initial="
                            << guideCompactBackendResult
                                   .corridor_slack_initial

                            << " slack_final="
                            << guideCompactBackendResult
                                   .corridor_slack_final

                            << " baseline_firi_faces="
                            << record.total_faces

                            << " baseline_firi_cost="
                            << record.final_cost

                            << " baseline_firi_setup_ms="
                            << record.optimizer_setup_ms

                            << " baseline_firi_opt_ms="
                            << record.optimizer_ms

                            << " exact_mapping_valid="
                            << guideCompactBackendResult
                                   .exact_mapping_valid

                            << " exact_cert_valid="
                            << guideCompactBackendResult
                                   .exact_certificate_valid

                            << " exact_contained="
                            << guideCompactBackendResult
                                   .exact_contained

                            << " exact_faces_checked="
                            << guideCompactBackendResult
                                   .exact_checked_faces

                            << " exact_max_violation_m="
                            << guideCompactBackendResult
                                   .exact_max_violation_m

                            << " exact_min_margin_m="
                            << guideCompactBackendResult
                                   .exact_min_margin_m

                            << " exact_worst_piece="
                            << guideCompactBackendResult
                                   .exact_worst_piece

                            << " exact_worst_face="
                            << guideCompactBackendResult
                                   .exact_worst_face

                            << " exact_worst_tau="
                            << guideCompactBackendResult
                                   .exact_worst_tau

                            << " exact_worst_t="
                            << guideCompactBackendResult
                                   .exact_worst_t

                            << " exact_cert_ms="
                            << guideCompactBackendResult
                                   .exact_certificate_ms);
                        }
                        ROS_INFO_STREAM(
                            "TF_GUIDE_ACTIVE_BACKEND "
                            << "corridor_success="
                            << activeGuideSuccess

                            << " setup_success="
                            << activeGuideBackendResult
                                   .setup_success

                            << " opt_success="
                            << activeGuideBackendResult
                                   .optimize_success

                            << " corridors="
                            << activeGuideBackendResult
                                   .corridor_count

                            << " faces="
                            << activeGuideBackendResult
                                   .total_faces

                            << " traj_pieces="
                            << activeGuideBackendResult
                                   .trajectory_pieces

                            << " constrained_pieces="
                            << activeGuideBackendResult
                                   .constrained_pieces

                            << " final_cost="
                            << activeGuideBackendResult
                                   .final_cost

                            << " duration="
                            << activeGuideBackendResult
                                   .trajectory_duration

                            << " setup_ms="
                            << activeGuideBackendResult
                                   .setup_ms

                            << " opt_ms="
                            << activeGuideBackendResult
                                   .optimize_ms

                            << " penalty_initial="
                            << activeGuideBackendResult
                                   .corridor_penalty_initial

                            << " penalty_final="
                            << activeGuideBackendResult
                                   .corridor_penalty_final

                            << " violation_initial="
                            << activeGuideBackendResult
                                   .max_corridor_violation_initial

                            << " violation_final="
                            << activeGuideBackendResult
                                   .max_corridor_violation_final

                            << " slack_initial="
                            << activeGuideBackendResult
                                   .corridor_slack_initial

                            << " slack_final="
                            << activeGuideBackendResult
                                   .corridor_slack_final

                            << " batch_corridor_success="
                            << guideCompactSuccess

                            << " batch_faces="
                            << guideCompactBackendResult
                                   .total_faces

                            << " batch_final_cost="
                            << guideCompactBackendResult
                                   .final_cost

                            << " batch_setup_ms="
                            << guideCompactBackendResult
                                   .setup_ms

                            << " batch_opt_ms="
                            << guideCompactBackendResult
                                   .optimize_ms

                            << " exact_mapping_valid="
                            << activeGuideBackendResult
                                   .exact_mapping_valid

                            << " exact_cert_valid="
                            << activeGuideBackendResult
                                   .exact_certificate_valid

                            << " exact_contained="
                            << activeGuideBackendResult
                                   .exact_contained

                            << " exact_faces_checked="
                            << activeGuideBackendResult
                                   .exact_checked_faces

                            << " exact_max_violation_m="
                            << activeGuideBackendResult
                                   .exact_max_violation_m

                            << " exact_min_margin_m="
                            << activeGuideBackendResult
                                   .exact_min_margin_m

                            << " exact_worst_piece="
                            << activeGuideBackendResult
                                   .exact_worst_piece

                            << " exact_worst_face="
                            << activeGuideBackendResult
                                   .exact_worst_face

                            << " exact_worst_tau="
                            << activeGuideBackendResult
                                   .exact_worst_tau

                            << " exact_worst_t="
                            << activeGuideBackendResult
                                   .exact_worst_t

                            << " exact_cert_ms="
                            << activeGuideBackendResult
                                   .exact_certificate_ms);

                    ROS_INFO_STREAM(
                        "TF_EXACT_HARD_SFC "
                        // ---- 源轨迹信息（来自 runBackendAb） ----
                        << " source_backend_ready="
                        << hardProjectionSourceReady
                        << " source_opt_ms="
                        << activeGuideBackendResult.optimize_ms
                        << " source_cert_mismatch_m="
                        << hardSourceCertificateMismatchM
                    
                        // ---- 硬投影核心结果 ----
                        << " projection_success="
                        << hardProjectionResult.success
                        << " affine_valid="
                        << hardProjectionResult.affine_map_valid
                    
                        << " initial_cert_valid="
                        << hardProjectionResult.initial_certificate_valid
                        << " initial_contained="
                        << hardProjectionResult.initial_contained
                        << " final_cert_valid="
                        << hardProjectionResult.final_certificate_valid
                        << " final_contained="
                        << hardProjectionResult.final_contained
                    
                        << " initial_violation_m="
                        << hardProjectionResult.initial_max_violation_m
                        << " final_violation_m="
                        << hardProjectionResult.final_max_violation_m
                    
                        // ---- 硬投影优化细节 ----
                        << " exchange_iterations="
                        << hardProjectionResult.exchange_iterations
                        << " active_constraints="
                        << hardProjectionResult.active_constraint_count
                        << " qp_sweeps="
                        << hardProjectionResult.total_qp_sweeps
                        << " duplicate_witnesses="
                        << hardProjectionResult.duplicate_witness_count
                    
                        << " correction_l2_m="
                        << hardProjectionResult.correction_l2_m
                        << " max_waypoint_disp_m="
                        << hardProjectionResult.max_waypoint_displacement_m
                    
                        // ---- 能量与时耗 ----
                        << " initial_energy="
                        << hardProjectionResult.initial_energy
                        << " final_energy="
                        << hardProjectionResult.final_energy
                    
                        << " projection_ms="
                        << hardProjectionResult.total_ms
                        << " qp_ms="
                        << hardProjectionResult.qp_ms
                        << " cert_ms="
                        << hardProjectionResult.certificate_ms);

                        // ============================================================
                        // In paper benchmark mode the externally visible trajectory
                        // must be the FINAL proposed trajectory, i.e. the trajectory
                        // after exact continuous-time SFC closure.
                        //
                        // Legacy debug mode keeps the historical baseline trajectory
                        // untouched so old A/B diagnostics remain reproducible.
                        // ============================================================
                        
                        const double proposedHardAfterRouteMs =
                            routeMincoGuideBuildMs +
                            guideMetricMs +
                            activeGuideMs +
                            activeGuideBackendResult.setup_ms +
                            activeGuideBackendResult.optimize_ms +
                            hardProjectionResult.total_ms;

                        ROS_INFO_STREAM(
                            "TF_PROPOSED_HARD_TIMING "
                            << "guide_ms="
                            << routeMincoGuideBuildMs
                        
                            << " csgn_ms="
                            << guideMetricMs
                        
                            << " corridor_ms="
                            << activeGuideMs
                        
                            << " setup_ms="
                            << activeGuideBackendResult
                                   .setup_ms
                        
                            << " optimize_ms="
                            << activeGuideBackendResult
                                   .optimize_ms
                        
                            << " hard_projection_ms="
                            << hardProjectionResult
                                   .total_ms
                        
                            << " after_route_ms="
                            << proposedHardAfterRouteMs
                        
                            << " baseline_after_route_ms="
                            << (record.corridor_generation_ms +
                                record.optimizer_setup_ms +
                                record.optimizer_ms)
                            
                            << " exact_feasible="
                            << hardProjectionResult
                                   .success);

                        if (legacyDebugMode)
                        {
                        ROS_INFO_STREAM(
                            "TF_GUIDE_ACTIVE_PENALTY_DIAG "
                            << "base_exact_violation="
                            << activeGuideBackendResult.exact_max_violation_m
                            << " x10_exact_violation="
                            << activeGuidePenalty10Result.exact_max_violation_m
                            << " base_exact_contained="
                            << activeGuideBackendResult.exact_contained
                            << " x10_exact_contained="
                            << activeGuidePenalty10Result.exact_contained
                            << " base_cost="
                            << activeGuideBackendResult.final_cost
                            << " x10_cost="
                            << activeGuidePenalty10Result.final_cost
                            << " base_opt_ms="
                            << activeGuideBackendResult.optimize_ms
                            << " x10_opt_ms="
                            << activeGuidePenalty10Result.optimize_ms);

                        ROS_INFO_STREAM(
                            "TF_GUIDE_ACTIVE_WARM_CONTINUATION "
                            << "setup_success="
                            << warmContinuationSetupSuccess

                            << " base_success="
                            << warmContinuationBaseSuccess

                            << " continuation_success="
                            << warmContinuationSuccess

                            << " exact_valid="
                            << warmContinuationExactValid

                            << " exact_contained="
                            << warmContinuationExactContained

                            << " exact_violation="
                            << warmContinuationExactViolation

                            << " base_ms="
                            << warmContinuationBaseMs

                            << " continuation_ms="
                            << warmContinuationMs

                            << " total_opt_ms="
                            << (warmContinuationBaseMs +
                                warmContinuationMs)

                            << " final_cost="
                            << warmContinuationFinalCost

                            << " cold_x10_opt_ms="
                            << activeGuidePenalty10Result
                                   .optimize_ms

                            << " cold_x10_cost="
                            << activeGuidePenalty10Result
                                   .final_cost);
                        }
                        if (legacyDebugMode)
                        {
                        ROS_INFO_STREAM(
                            "TF_GUIDE_PROPOSED_TIMING "
                            << "path_ms="
                            << record.path_search_ms

                            << " guide_build_ms="
                            << routeMincoGuideBuildMs

                            << " csgn_ms="
                            << guideMetricMs

                            << " corridor_ms="
                            << guideCompactMs

                            << " setup_ms="
                            << guideCompactBackendResult
                                   .setup_ms

                            << " opt_ms="
                            << guideCompactBackendResult
                                   .optimize_ms

                            << " proposed_after_route_component_ms="
                            << guideProposedComponentMs

                            << " baseline_after_route_ms="
                            << (record.corridor_generation_ms +
                                record.optimizer_setup_ms +
                                record.optimizer_ms));
                        }
                        ROS_INFO_STREAM(
                            "TF_GUIDE_ACTIVE_TIMING "
                            << "path_ms="
                            << record.path_search_ms

                            << " guide_build_ms="
                            << routeMincoGuideBuildMs

                            << " csgn_ms="
                            << guideMetricMs

                            << " corridor_ms="
                            << activeGuideMs

                            << " setup_ms="
                            << activeGuideBackendResult
                                   .setup_ms

                            << " opt_ms="
                            << activeGuideBackendResult
                                   .optimize_ms

                            << " proposed_after_route_component_ms="
                            << activeGuideProposedComponentMs

                            << " baseline_after_route_ms="
                            << (record.corridor_generation_ms +
                                record.optimizer_setup_ms +
                                record.optimizer_ms)

                            << " batch_after_route_component_ms="
                            << guideProposedComponentMs);

                        if (legacyDebugMode)
                        {
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
