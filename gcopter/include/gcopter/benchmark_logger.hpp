#ifndef GCOPTER_BENCHMARK_LOGGER_HPP
#define GCOPTER_BENCHMARK_LOGGER_HPP

#include <cerrno>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <cstdint>
#include <limits>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

namespace gcopter_benchmark
{

struct CaseRecord
{
    std::string case_id;

    std::string environment_family =
        "unknown";

    std::string difficulty =
        "unknown";

    std::string map_id;

    int map_seed =
        0;

    std::string route_id;

    int source_route_seed =
        0;

    std::string route_file;

    std::string route_fingerprint;

    double route_length_m =
        0.0;

    int route_point_count =
        0;

    int route_segment_count =
        0;

    double start_x =
        0.0;

    double start_y =
        0.0;

    double start_z =
        0.0;

    double goal_x =
        0.0;

    double goal_y =
        0.0;

    double goal_z =
        0.0;

    double voxel_width_m =
        0.0;

    double dilate_radius_m =
        0.0;

    int map_point_count =
        -1;

    bool route_validation_success =
        false;

    int route_validation_samples =
        0;

    double max_route_segment_m =
        0.0;

    double creation_timestamp_s =
        0.0;
};

struct BenchmarkRunRecord
{
    // ========================================================
    // Identity
    // ========================================================
    std::string case_id;

    std::string route_fingerprint;

    std::string method =
        "unknown";

    std::string variant =
        "unknown";

    int repeat_id =
        0;

    double timestamp_s =
        0.0;

    // ========================================================
    // Pipeline status
    // ========================================================
    bool corridor_success =
        false;

    bool optimizer_setup_success =
        false;

    bool optimizer_success =
        false;

    bool final_success =
        false;

    // ========================================================
    // Corridor complexity / guarantees
    // ========================================================
    int corridor_count =
        0;

    int total_faces =
        0;

    int obstacle_faces =
        0;

    int domain_faces =
        0;

    int safety_valid_count =
        0;

    int safety_total_count =
        0;

    int overlap_valid_count =
        0;

    int overlap_total_count =
        0;

    // ========================================================
    // Active-Witness workload
    // ========================================================
    std::int64_t candidate_count =
        0;

    std::int64_t active_witness_rounds =
        0;

    std::int64_t witness_distance_tests =
        0;

    std::int64_t obstacle_face_tests =
        0;

    int redundancy_removed =
        0;

    // ========================================================
    // Proposed pipeline timing.
    //
    // IMPORTANT:
    // after_route_ms excludes RRT and all debug-only A/B work.
    // ========================================================
    double guide_ms =
        0.0;

    double csgn_ms =
        0.0;

    double corridor_ms =
        0.0;

    double setup_ms =
        0.0;

    double optimize_ms =
        0.0;

    double hard_projection_ms =
        0.0;

    double after_route_ms =
        0.0;

    // ========================================================
    // Backend trajectory
    // ========================================================
    int trajectory_piece_count =
        0;

    double trajectory_duration_s =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_optimizer_cost =
        std::numeric_limits<double>::
            quiet_NaN();

    // ========================================================
    // Final trajectory quality / dynamics.
    //
    // These fields describe the FINAL trajectory after exact
    // SFC closure.
    // ========================================================
    bool trajectory_metrics_valid =
        false;

    double trajectory_length_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double smoothness_energy =
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


    // ========================================================
    // Soft source trajectory quality / dynamics.
    //
    // These fields describe the SAME GCOPTER optimum consumed
    // by the exact-hard closure, before waypoint projection.
    // ========================================================
    bool soft_trajectory_metrics_valid =
        false;

    double soft_trajectory_duration_s =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_trajectory_length_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_smoothness_energy =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_time_cost =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_j_kin =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_max_velocity_mps =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_max_acceleration_mps2 =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_max_body_rate_radps =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_max_tilt_rad =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_min_thrust_n =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_max_thrust_n =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Soft-source reconstruction provenance.
    // ========================================================
    bool soft_hard_comparison_valid =
        false;

    double soft_rebuild_energy_delta =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_rebuild_duration_delta_s =
        std::numeric_limits<double>::
            quiet_NaN();

    // ========================================================
    // Continuous-time SFC safety before hard closure
    // ========================================================
    bool soft_exact_certificate_valid =
        false;

    bool soft_exact_contained =
        false;

    double soft_exact_max_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();

    // ========================================================
    // Exact-support hard closure
    // ========================================================
    bool hard_projection_triggered =
        false;

    int exchange_iterations =
        0;

    int active_time_constraints =
        0;

    int qp_sweeps =
        0;

    bool final_exact_certificate_valid =
        false;

    bool final_exact_contained =
        false;

    double final_exact_max_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double correction_l2_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_waypoint_disp_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double energy_before =
        std::numeric_limits<double>::
            quiet_NaN();

    double energy_after =
        std::numeric_limits<double>::
            quiet_NaN();
};

struct BenchmarkControlledE2Record
{
    // ========================================================
    // Identity / pairing
    // ========================================================
    std::string case_id;

    std::string route_fingerprint;

    std::string method =
        "unknown";

    std::string variant =
        "unknown";

    int repeat_id =
        0;

    double timestamp_s =
        0.0;

    std::string protocol =
        "controlled_geometry_common_soft_gcopter";


    // ========================================================
    // Common-backend status
    // ========================================================
    bool mapping_valid =
        false;

    bool backend_attempted =
        false;

    bool setup_success =
        false;

    bool optimize_success =
        false;

    bool optimized_state_ready =
        false;

    bool trajectory_rebuild_ready =
        false;

    bool trajectory_metrics_valid =
        false;


    // ========================================================
    // Corridor / backend workload
    //
    // setup_ms / optimize_ms here are descriptive E2 backend
    // measurements. They are NOT the final E4 timing protocol.
    // ========================================================
    int corridor_count =
        0;

    int raw_face_count =
        0;

    int trajectory_piece_count =
        0;

    // ========================================================
    // Optimizer workload instrumentation.
    //
    // These fields are serialized ONLY to
    // benchmark_e2_workload_v1.csv.
    //
    // benchmark_e2_v1.csv remains schema-1 and unchanged.
    // ========================================================
    int temporal_variable_dim =
        0;

    int spatial_variable_dim =
        0;

    int optimizer_variable_dim =
        0;

    int quadrature_nodes_per_piece =
        0;

    std::int64_t objective_evaluation_count =
        0;

    std::int64_t geometric_face_evaluations_per_objective =
        0;

    std::int64_t total_geometric_face_evaluations =
        0;

    double setup_ms =
        std::numeric_limits<double>::
            quiet_NaN();

    double optimize_ms =
        std::numeric_limits<double>::
            quiet_NaN();

    double optimizer_cost =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Common soft-trajectory quality
    // ========================================================
    double duration_s =
        std::numeric_limits<double>::
            quiet_NaN();

    double length_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double smoothness_energy =
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


    // ========================================================
    // Reconstruction provenance
    // ========================================================
    double rebuild_duration_delta_s =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Exact continuous-time certificate of the SOFT optimum
    // ========================================================
    bool soft_exact_mapping_valid =
        false;

    bool soft_exact_certificate_valid =
        false;

    bool soft_exact_contained =
        false;

    double soft_exact_max_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_exact_min_margin_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double soft_exact_certificate_ms =
        std::numeric_limits<double>::
            quiet_NaN();
};

struct BenchmarkCorridorRecord
{
    // ========================================================
    // Identity
    // ========================================================
    std::string case_id;

    std::string route_fingerprint;

    std::string method =
        "unknown";

    std::string variant =
        "unknown";

    int repeat_id =
        0;

    double timestamp_s =
        0.0;

    int corridor_id =
        -1;

    int source_segment_id =
        -1;

    bool geometry_mapping_valid =
        false;

    // ========================================================
    // Geometry protocol / directional-basis provenance
    // ========================================================
    std::string geometry_protocol =
        "unknown";

    std::string construction_direction_basis =
        "unknown";

    std::string reference_direction_source =
        "unknown";

    // ========================================================
    // Protected seed / neighboring connectivity
    // ========================================================
    bool seed_metric_valid =
        false;

    double seed_radius_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double protected_radius_m =
        std::numeric_limits<double>::
            quiet_NaN();

    bool junction_overlap_valid =
        false;

    double junction_overlap_radius_m =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Face complexity
    // ========================================================
    int total_faces =
        0;

    int domain_faces =
        0;

    int obstacle_faces =
        0;


    // ========================================================
    // Active-Witness / constraint-generation workload
    // ========================================================
    int input_obstacle_count =
        0;

    int local_obstacle_count =
        0;

    std::int64_t candidate_count =
        0;

    std::int64_t generated_candidate_count =
        0;

    std::int64_t active_witness_rounds =
        0;

    std::int64_t witness_distance_tests =
        0;

    std::int64_t obstacle_face_tests =
        0;

    int greedy_obstacle_face_count =
        0;

    int redundancy_removed =
        0;

    bool safety_verified =
        false;

    bool overlap_guaranteed =
        false;


    // ========================================================
    // CSGN / trajectory-relevance diagnostics
    // ========================================================
    bool metric_valid =
        false;

    bool anisotropic_domain =
        false;

    double utility_eig0 =
        std::numeric_limits<double>::
            quiet_NaN();

    double utility_eig1 =
        std::numeric_limits<double>::
            quiet_NaN();

    double utility_eig2 =
        std::numeric_limits<double>::
            quiet_NaN();

    double utility_anisotropy =
        std::numeric_limits<double>::
            quiet_NaN();

    // IMPORTANT:
    // These are construction-domain allowances, NOT measured
    // final-polytope directional widths.
    double construction_extra_radius0_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double construction_extra_radius1_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double construction_extra_radius2_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double mean_metric_damage =
        std::numeric_limits<double>::
            quiet_NaN();

    double min_metric_damage =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_metric_damage =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Direct-guide MINCO -> route-segment mapping provenance
    // ========================================================
    int metric_source_piece_id =
        -1;

    double metric_mapping_distance =
        std::numeric_limits<double>::
            quiet_NaN();

    // ========================================================
    // Reference CSGN metric used ONLY to define the common
    // measurement directions.
    //
    // This is separate from the metric actually used to BUILD
    // this particular corridor.
    //
    // Example:
    //   CSGN row:
    //       construction metric = CSGN
    //       reference metric    = CSGN
    //
    //   Identity row:
    //       construction metric = Identity / disabled
    //       reference metric    = CSGN
    // ========================================================
    bool reference_metric_valid =
        false;

    double reference_utility_eig0 =
        std::numeric_limits<double>::
            quiet_NaN();

    double reference_utility_eig1 =
        std::numeric_limits<double>::
            quiet_NaN();

    double reference_utility_eig2 =
        std::numeric_limits<double>::
            quiet_NaN();

    int reference_metric_source_piece_id =
        -1;

    double reference_metric_mapping_distance =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Final-polytope protected-seed directional reserve,
    // measured along the COMMON CSGN reference directions.
    //
    // No preservation ratios are stored here.
    // ========================================================
    bool directional_reserve_valid =
        false;

    double hard_positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double hard_negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double hard_symmetric_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double hard_span_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_symmetric_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_span_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_symmetric_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_span_m =
        std::numeric_limits<double>::
            quiet_NaN();

};

struct BenchmarkControlledCorridorRecordV3
{
    // ========================================================
    // Identity / pairing provenance
    // ========================================================
    std::string case_id;

    std::string route_fingerprint;

    std::string method =
        "unknown";

    std::string variant =
        "unknown";

    int repeat_id =
        0;

    double timestamp_s =
        0.0;

    int corridor_id =
        -1;

    int source_segment_id =
        -1;

    bool geometry_mapping_valid =
        false;

    std::string geometry_protocol =
        "controlled_geometry";


    // ========================================================
    // Construction provenance
    // ========================================================
    std::string construction_algorithm =
        "unknown";

    std::string construction_direction_basis =
        "unknown";

    std::string reference_direction_source =
        "direct_minco_csgn";

    int input_obstacle_count =
        0;

    int local_obstacle_count =
        -1;

    // Actual H rows stored / consumed downstream.
    int raw_face_count =
        0;

    int raw_domain_face_count =
        0;

    int raw_obstacle_face_count =
        0;


    // Active-Witness-only workload.
    //
    // -1 means not applicable for this method.
    std::int64_t aw_candidate_count =
        -1;

    std::int64_t aw_active_rounds =
        -1;

    std::int64_t aw_witness_distance_tests =
        -1;

    std::int64_t aw_obstacle_face_tests =
        -1;


    // ========================================================
    // COMMON independent safety measurement
    // ========================================================
    bool common_safety_valid =
        false;

    bool common_safe =
        false;

    bool obstacle_surface_safe =
        false;

    bool map_contained =
        false;

    int obstacle_sample_count =
        0;

    int worst_obstacle_index =
        -1;

    double min_obstacle_exclusion_margin_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_obstacle_penetration_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double max_map_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // Seed / neighboring overlap geometry
    // ========================================================
    bool seed_metric_valid =
        false;

    double seed_radius_m =
        std::numeric_limits<double>::
            quiet_NaN();

    // True only when the construction explicitly prescribes
    // a protected finite radius as part of the algorithm.
    //
    // CSGN / Identity compact ablation:
    //     true
    //
    // FIRI / RILS:
    //     false
    bool protected_radius_prescribed =
        false;

    double prescribed_protected_radius_m =
        std::numeric_limits<double>::
            quiet_NaN();

    bool prescribed_seed_satisfied =
        false;

    bool junction_overlap_valid =
        false;

    double junction_overlap_radius_m =
        std::numeric_limits<double>::
            quiet_NaN();

    bool prescribed_overlap_satisfied =
        false;


    // ========================================================
    // COMMON Direct-MINCO CSGN reference basis
    // ========================================================
    bool reference_metric_valid =
        false;

    double reference_utility_eig0 =
        std::numeric_limits<double>::
            quiet_NaN();

    double reference_utility_eig1 =
        std::numeric_limits<double>::
            quiet_NaN();

    double reference_utility_eig2 =
        std::numeric_limits<double>::
            quiet_NaN();

    int reference_metric_source_piece_id =
        -1;

    double reference_metric_mapping_distance =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // COMMON midpoint directional widths
    //
    // Reference point:
    //
    //     q_i = 0.5 * (route[i] + route[i+1])
    //
    // Directions:
    //
    //     eig0 = hard
    //     eig1 = middle
    //     eig2 = easy
    // ========================================================
    bool point_width_valid =
        false;

    double point_reference_margin_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double hard_positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double hard_negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double hard_width_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double middle_width_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_positive_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_negative_m =
        std::numeric_limits<double>::
            quiet_NaN();

    double easy_width_m =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // COMMON final-polytope volume
    // ========================================================
    bool volume_valid =
        false;

    double volume_m3 =
        std::numeric_limits<double>::
            quiet_NaN();

    int volume_vertex_count =
        0;

    int volume_triangle_count =
        0;

    double volume_max_vertex_violation_m =
        std::numeric_limits<double>::
            quiet_NaN();


    // ========================================================
    // COMMON LP-based effective-face measurement
    // ========================================================
    bool effective_face_valid =
        false;

    int unique_plane_groups =
        0;

    int duplicate_rows =
        0;

    int effective_face_count =
        0;

    int redundant_plane_groups =
        0;
};

class CaseCsvLogger
{
private:
    bool enabled_;

    std::string directory_;

    std::mutex mutex_;


    static inline bool
    fileNeedsHeader(
        const std::string &path)
    {
        std::ifstream input(
            path,
            std::ios::binary);

        return
            !input ||
            input.peek() ==
                std::ifstream::
                    traits_type::eof();
    }


    static inline std::string
    csv(
        const std::string &value)
    {
        if (value.find_first_of(
                ",\"\n\r") ==
            std::string::npos)
        {
            return value;
        }

        std::string escaped =
            "\"";

        for (const char c :
             value)
        {
            escaped +=
                c;

            if (c == '"')
            {
                escaped +=
                    '"';
            }
        }

        escaped +=
            '"';

        return escaped;
    }


    inline bool
    ensureDirectory() const
    {
        if (directory_.empty())
        {
            return false;
        }

        std::string current =
            directory_.front() == '/'
                ? "/"
                : "";

        std::istringstream path(
            directory_);

        std::string part;

        while (std::getline(
            path,
            part,
            '/'))
        {
            if (part.empty())
            {
                continue;
            }

            if (!current.empty() &&
                current.back() != '/')
            {
                current += '/';
            }

            current +=
                part;

            if (::mkdir(
                    current.c_str(),
                    0755) != 0 &&
                errno != EEXIST)
            {
                return false;
            }
        }

        return true;
    }


public:
    CaseCsvLogger(
        const bool enabled,
        const std::string &directory)
        : enabled_(enabled),
          directory_(directory)
    {
        while (directory_.size() > 1 &&
               directory_.back() == '/')
        {
            directory_.pop_back();
        }
    }


    inline bool logCase(
        const CaseRecord &record)
    {
        if (!enabled_)
        {
            return true;
        }

        std::lock_guard<
            std::mutex>
            lock(
                mutex_);

        if (!ensureDirectory())
        {
            return false;
        }

        const std::string path =
            directory_ +
            "/benchmark_cases_v1.csv";

        const bool header =
            fileNeedsHeader(
                path);

        std::ofstream output(
            path,
            std::ios::out |
                std::ios::app);

        if (!output)
        {
            return false;
        }

        if (header)
        {
            output
                << "schema_version,"
                << "case_id,"
                << "environment_family,"
                << "difficulty,"
                << "map_id,"
                << "map_seed,"
                << "route_id,"
                << "source_route_seed,"
                << "route_file,"
                << "route_fingerprint,"
                << "route_length_m,"
                << "route_point_count,"
                << "route_segment_count,"
                << "start_x,start_y,start_z,"
                << "goal_x,goal_y,goal_z,"
                << "voxel_width_m,"
                << "dilate_radius_m,"
                << "map_point_count,"
                << "route_validation_success,"
                << "route_validation_samples,"
                << "max_route_segment_m,"
                << "creation_timestamp_s\n";
        }

        output
            << std::setprecision(17)

            << 1
            << ','

            << csv(
                   record.case_id)
            << ','

            << csv(
                   record
                       .environment_family)
            << ','

            << csv(
                   record.difficulty)
            << ','

            << csv(
                   record.map_id)
            << ','

            << record.map_seed
            << ','

            << csv(
                   record.route_id)
            << ','

            << record
                   .source_route_seed
            << ','

            << csv(
                   record.route_file)
            << ','

            << csv(
                   record
                       .route_fingerprint)
            << ','

            << record
                   .route_length_m
            << ','

            << record
                   .route_point_count
            << ','

            << record
                   .route_segment_count
            << ','

            << record.start_x
            << ','
            << record.start_y
            << ','
            << record.start_z
            << ','

            << record.goal_x
            << ','
            << record.goal_y
            << ','
            << record.goal_z
            << ','

            << record
                   .voxel_width_m
            << ','

            << record
                   .dilate_radius_m
            << ','

            << record
                   .map_point_count
            << ','

            << record
                   .route_validation_success
            << ','

            << record
                   .route_validation_samples
            << ','

            << record
                   .max_route_segment_m
            << ','

            << record
                   .creation_timestamp_s
            << '\n';

        return static_cast<bool>(
            output);
    }
};

class RunCsvLogger
{
private:
    bool enabled_;

    std::string directory_;

    std::mutex mutex_;


    static inline bool
    fileNeedsHeader(
        const std::string &path)
    {
        std::ifstream input(
            path,
            std::ios::binary);

        return
            !input ||
            input.peek() ==
                std::ifstream::
                    traits_type::eof();
    }


    static inline std::string
    csv(
        const std::string &value)
    {
        if (value.find_first_of(
                ",\"\n\r") ==
            std::string::npos)
        {
            return value;
        }

        std::string escaped =
            "\"";

        for (const char c :
             value)
        {
            escaped +=
                c;

            if (c == '"')
            {
                escaped +=
                    '"';
            }
        }

        escaped +=
            '"';

        return escaped;
    }


    inline bool
    ensureDirectory() const
    {
        if (directory_.empty())
        {
            return false;
        }

        std::string current =
            directory_.front() == '/'
                ? "/"
                : "";

        std::istringstream path(
            directory_);

        std::string part;

        while (std::getline(
            path,
            part,
            '/'))
        {
            if (part.empty())
            {
                continue;
            }

            if (!current.empty() &&
                current.back() != '/')
            {
                current += '/';
            }

            current +=
                part;

            if (::mkdir(
                    current.c_str(),
                    0755) != 0 &&
                errno != EEXIST)
            {
                return false;
            }
        }

        return true;
    }


public:
    RunCsvLogger(
        const bool enabled,
        const std::string &directory)
        : enabled_(enabled),
          directory_(directory)
    {
        while (directory_.size() > 1 &&
               directory_.back() == '/')
        {
            directory_.pop_back();
        }
    }


    inline bool logRun(
        const BenchmarkRunRecord &record)
    {
        if (!enabled_)
        {
            return true;
        }

        std::lock_guard<std::mutex>
            lock(
                mutex_);

        if (!ensureDirectory())
        {
            return false;
        }

        const std::string path =
            directory_ +
            "/benchmark_runs_v2.csv";

        const bool header =
            fileNeedsHeader(
                path);

        std::ofstream output(
            path,
            std::ios::out |
                std::ios::app);

        if (!output)
        {
            return false;
        }

        if (header)
        {
            output
                << "schema_version,"
                << "case_id,"
                << "route_fingerprint,"
                << "method,"
                << "variant,"
                << "repeat_id,"
                << "timestamp_s,"

                << "corridor_success,"
                << "optimizer_setup_success,"
                << "optimizer_success,"
                << "final_success,"

                << "corridor_count,"
                << "total_faces,"
                << "obstacle_faces,"
                << "domain_faces,"
                << "safety_valid_count,"
                << "safety_total_count,"
                << "overlap_valid_count,"
                << "overlap_total_count,"

                << "candidate_count,"
                << "active_witness_rounds,"
                << "witness_distance_tests,"
                << "obstacle_face_tests,"
                << "redundancy_removed,"

                << "guide_ms,"
                << "csgn_ms,"
                << "corridor_ms,"
                << "setup_ms,"
                << "optimize_ms,"
                << "hard_projection_ms,"
                << "after_route_ms,"

                << "trajectory_piece_count,"
                << "trajectory_duration_s,"
                << "soft_optimizer_cost,"

                // Final trajectory metrics
                << "trajectory_metrics_valid,"
                << "trajectory_length_m,"
                << "smoothness_energy,"
                << "time_cost,"
                << "j_kin,"
                << "max_velocity_mps,"
                << "max_acceleration_mps2,"
                << "max_body_rate_radps,"
                << "max_tilt_rad,"
                << "min_thrust_n,"
                << "max_thrust_n,"

                // Soft source trajectory metrics
                << "soft_trajectory_metrics_valid,"
                << "soft_trajectory_duration_s,"
                << "soft_trajectory_length_m,"
                << "soft_smoothness_energy,"
                << "soft_time_cost,"
                << "soft_j_kin,"
                << "soft_max_velocity_mps,"
                << "soft_max_acceleration_mps2,"
                << "soft_max_body_rate_radps,"
                << "soft_max_tilt_rad,"
                << "soft_min_thrust_n,"
                << "soft_max_thrust_n,"

                // Provenance
                << "soft_hard_comparison_valid,"
                << "soft_rebuild_energy_delta,"
                << "soft_rebuild_duration_delta_s,"

                << "soft_exact_certificate_valid,"
                << "soft_exact_contained,"
                << "soft_exact_max_violation_m,"

                << "hard_projection_triggered,"
                << "exchange_iterations,"
                << "active_time_constraints,"
                << "qp_sweeps,"

                << "final_exact_certificate_valid,"
                << "final_exact_contained,"
                << "final_exact_max_violation_m,"

                << "correction_l2_m,"
                << "max_waypoint_disp_m,"
                << "energy_before,"
                << "energy_after\n";
        }

        output
            << std::setprecision(17)

            << 2 << ','

            << csv(record.case_id) << ','
            << csv(record.route_fingerprint) << ','
            << csv(record.method) << ','
            << csv(record.variant) << ','
            << record.repeat_id << ','
            << record.timestamp_s << ','

            << record.corridor_success << ','
            << record.optimizer_setup_success << ','
            << record.optimizer_success << ','
            << record.final_success << ','

            << record.corridor_count << ','
            << record.total_faces << ','
            << record.obstacle_faces << ','
            << record.domain_faces << ','
            << record.safety_valid_count << ','
            << record.safety_total_count << ','
            << record.overlap_valid_count << ','
            << record.overlap_total_count << ','

            << record.candidate_count << ','
            << record.active_witness_rounds << ','
            << record.witness_distance_tests << ','
            << record.obstacle_face_tests << ','
            << record.redundancy_removed << ','

            << record.guide_ms << ','
            << record.csgn_ms << ','
            << record.corridor_ms << ','
            << record.setup_ms << ','
            << record.optimize_ms << ','
            << record.hard_projection_ms << ','
            << record.after_route_ms << ','

            << record.trajectory_piece_count << ','
            << record.trajectory_duration_s << ','
            << record.soft_optimizer_cost << ','
                
            // Final trajectory metrics
            << record.trajectory_metrics_valid << ','
            << record.trajectory_length_m << ','
            << record.smoothness_energy << ','
            << record.time_cost << ','
            << record.j_kin << ','
            << record.max_velocity_mps << ','
            << record.max_acceleration_mps2 << ','
            << record.max_body_rate_radps << ','
            << record.max_tilt_rad << ','
            << record.min_thrust_n << ','
            << record.max_thrust_n << ','
                
            // Soft source trajectory metrics
            << record.soft_trajectory_metrics_valid << ','
            << record.soft_trajectory_duration_s << ','
            << record.soft_trajectory_length_m << ','
            << record.soft_smoothness_energy << ','
            << record.soft_time_cost << ','
            << record.soft_j_kin << ','
            << record.soft_max_velocity_mps << ','
            << record.soft_max_acceleration_mps2 << ','
            << record.soft_max_body_rate_radps << ','
            << record.soft_max_tilt_rad << ','
            << record.soft_min_thrust_n << ','
            << record.soft_max_thrust_n << ','
                
            // Provenance
            << record.soft_hard_comparison_valid << ','
            << record.soft_rebuild_energy_delta << ','
            << record.soft_rebuild_duration_delta_s << ','

            << record.soft_exact_certificate_valid << ','
            << record.soft_exact_contained << ','
            << record.soft_exact_max_violation_m << ','

            << record.hard_projection_triggered << ','
            << record.exchange_iterations << ','
            << record.active_time_constraints << ','
            << record.qp_sweeps << ','

            << record.final_exact_certificate_valid << ','
            << record.final_exact_contained << ','
            << record.final_exact_max_violation_m << ','

            << record.correction_l2_m << ','
            << record.max_waypoint_disp_m << ','
            << record.energy_before << ','
            << record.energy_after
            << '\n';

        return static_cast<bool>(
            output);
    }

    inline bool
    logControlledE2(
        const std::vector<
            BenchmarkControlledE2Record> &records)
    {
        if (!enabled_ ||
            records.empty())
        {
            return true;
        }


        std::lock_guard<std::mutex>
            lock(
                mutex_);


        if (!ensureDirectory())
        {
            return false;
        }


        const std::string path =
            directory_ +
            "/benchmark_e2_v1.csv";


        const bool header =
            fileNeedsHeader(
                path);


        std::ofstream output(
            path,
            std::ios::out |
                std::ios::app);


        if (!output)
        {
            return false;
        }


        if (header)
        {
            output
                << "schema_version,"

                << "case_id,"
                << "route_fingerprint,"
                << "method,"
                << "variant,"
                << "repeat_id,"
                << "timestamp_s,"
                << "protocol,"

                << "mapping_valid,"
                << "backend_attempted,"
                << "setup_success,"
                << "optimize_success,"
                << "optimized_state_ready,"
                << "trajectory_rebuild_ready,"
                << "trajectory_metrics_valid,"

                << "corridor_count,"
                << "raw_face_count,"
                << "trajectory_piece_count,"

                << "setup_ms,"
                << "optimize_ms,"
                << "optimizer_cost,"

                << "duration_s,"
                << "length_m,"
                << "smoothness_energy,"
                << "time_cost,"
                << "j_kin,"

                << "max_velocity_mps,"
                << "max_acceleration_mps2,"
                << "max_body_rate_radps,"
                << "max_tilt_rad,"
                << "min_thrust_n,"
                << "max_thrust_n,"

                << "rebuild_duration_delta_s,"

                << "soft_exact_mapping_valid,"
                << "soft_exact_certificate_valid,"
                << "soft_exact_contained,"
                << "soft_exact_max_violation_m,"
                << "soft_exact_min_margin_m,"
                << "soft_exact_certificate_ms\n";
        }


        output <<
            std::setprecision(17);


        for (const auto &record :
             records)
        {
            output
                << 1 << ','

                << csv(record.case_id) << ','
                << csv(record.route_fingerprint) << ','
                << csv(record.method) << ','
                << csv(record.variant) << ','
                << record.repeat_id << ','
                << record.timestamp_s << ','
                << csv(record.protocol) << ','

                << record.mapping_valid << ','
                << record.backend_attempted << ','
                << record.setup_success << ','
                << record.optimize_success << ','
                << record.optimized_state_ready << ','
                << record.trajectory_rebuild_ready << ','
                << record.trajectory_metrics_valid << ','

                << record.corridor_count << ','
                << record.raw_face_count << ','
                << record.trajectory_piece_count << ','

                << record.setup_ms << ','
                << record.optimize_ms << ','
                << record.optimizer_cost << ','

                << record.duration_s << ','
                << record.length_m << ','
                << record.smoothness_energy << ','
                << record.time_cost << ','
                << record.j_kin << ','

                << record.max_velocity_mps << ','
                << record.max_acceleration_mps2 << ','
                << record.max_body_rate_radps << ','
                << record.max_tilt_rad << ','
                << record.min_thrust_n << ','
                << record.max_thrust_n << ','

                << record.rebuild_duration_delta_s << ','

                << record.soft_exact_mapping_valid << ','
                << record.soft_exact_certificate_valid << ','
                << record.soft_exact_contained << ','
                << record.soft_exact_max_violation_m << ','
                << record.soft_exact_min_margin_m << ','
                << record.soft_exact_certificate_ms

                << '\n';
        }

        return static_cast<bool>(
            output);
    }


    // ========================================================
    // P1: downstream optimizer workload instrumentation.
    //
    // Separate file by design:
    //
    //   benchmark_e2_v1.csv
    //
    // remains frozen and backward-compatible.
    // ========================================================
    inline bool
    logControlledE2Workload(
        const std::vector<
            BenchmarkControlledE2Record> &records)
    {
        if (!enabled_ ||
            records.empty())
        {
            return true;
        }

        std::lock_guard<std::mutex>
            lock(
                mutex_);

        if (!ensureDirectory())
        {
            return false;
        }

        const std::string path =
            directory_ +
            "/benchmark_e2_workload_v1.csv";

        const bool header =
            fileNeedsHeader(
                path);

        std::ofstream output(
            path,
            std::ios::out |
                std::ios::app);

        if (!output)
        {
            return false;
        }

        if (header)
        {
            output
                << "schema_version,"
                << "case_id,"
                << "route_fingerprint,"
                << "method,"
                << "variant,"
                << "repeat_id,"
                << "timestamp_s,"
                << "protocol,"
                << "mapping_valid,"
                << "backend_attempted,"
                << "setup_success,"
                << "optimize_success,"
                << "corridor_count,"
                << "raw_face_count,"
                << "trajectory_piece_count,"
                << "temporal_variable_dim,"
                << "spatial_variable_dim,"
                << "optimizer_variable_dim,"
                << "quadrature_nodes_per_piece,"
                << "objective_evaluation_count,"
                << "geometric_face_evaluations_per_objective,"
                << "total_geometric_face_evaluations,"
                << "setup_ms,"
                << "optimize_ms,"
                << "optimizer_cost\n";
        }

        output <<
            std::setprecision(17);

        for (const auto &record :
             records)
        {
            output
                << 1 << ','
                << csv(record.case_id) << ','
                << csv(record.route_fingerprint) << ','
                << csv(record.method) << ','
                << csv(record.variant) << ','
                << record.repeat_id << ','
                << record.timestamp_s << ','
                << csv(record.protocol) << ','

                << record.mapping_valid << ','
                << record.backend_attempted << ','
                << record.setup_success << ','
                << record.optimize_success << ','

                << record.corridor_count << ','
                << record.raw_face_count << ','
                << record.trajectory_piece_count << ','

                << record.temporal_variable_dim << ','
                << record.spatial_variable_dim << ','
                << record.optimizer_variable_dim << ','
                << record.quadrature_nodes_per_piece << ','

                << record.objective_evaluation_count << ','
                << record.geometric_face_evaluations_per_objective << ','
                << record.total_geometric_face_evaluations << ','

                << record.setup_ms << ','
                << record.optimize_ms << ','
                << record.optimizer_cost
                << '\n';
        }

        return static_cast<bool>(
            output);
    }

};

class CorridorCsvLogger
{
private:
    bool enabled_;

    std::string directory_;

    std::mutex mutex_;


    static inline bool
    fileNeedsHeader(
        const std::string &path)
    {
        std::ifstream input(
            path,
            std::ios::binary);

        return
            !input ||
            input.peek() ==
                std::ifstream::
                    traits_type::eof();
    }


    static inline std::string
    csv(
        const std::string &value)
    {
        if (value.find_first_of(
                ",\"\n\r") ==
            std::string::npos)
        {
            return value;
        }

        std::string escaped =
            "\"";

        for (const char c :
             value)
        {
            escaped += c;

            if (c == '"')
            {
                escaped += '"';
            }
        }

        escaped += '"';

        return escaped;
    }


    inline bool
    ensureDirectory() const
    {
        if (directory_.empty())
        {
            return false;
        }

        std::string current =
            directory_.front() == '/'
                ? "/"
                : "";

        std::istringstream path(
            directory_);

        std::string part;

        while (std::getline(
            path,
            part,
            '/'))
        {
            if (part.empty())
            {
                continue;
            }

            if (!current.empty() &&
                current.back() != '/')
            {
                current += '/';
            }

            current += part;

            if (::mkdir(
                    current.c_str(),
                    0755) != 0 &&
                errno != EEXIST)
            {
                return false;
            }
        }

        return true;
    }


public:
    CorridorCsvLogger(
        const bool enabled,
        const std::string &directory)
        : enabled_(enabled),
          directory_(directory)
    {
        while (directory_.size() > 1 &&
               directory_.back() == '/')
        {
            directory_.pop_back();
        }
    }


    inline bool logCorridors(
        const std::vector<
            BenchmarkCorridorRecord> &records)
    {
        if (!enabled_ ||
            records.empty())
        {
            return true;
        }

        std::lock_guard<std::mutex>
            lock(
                mutex_);

        if (!ensureDirectory())
        {
            return false;
        }

        const std::string path =
            directory_ +
            "/benchmark_corridors_v2.csv";

        const bool header =
            fileNeedsHeader(
                path);

        std::ofstream output(
            path,
            std::ios::out |
                std::ios::app);

        if (!output)
        {
            return false;
        }

        if (header)
        {
            output
                << "schema_version,"
                << "case_id,"
                << "route_fingerprint,"
                << "method,"
                << "variant,"
                << "repeat_id,"
                << "timestamp_s,"
                << "corridor_id,"
                << "source_segment_id,"
                << "geometry_mapping_valid,"
                << "geometry_protocol,"
                << "construction_direction_basis,"
                << "reference_direction_source,"

                << "seed_metric_valid,"
                << "seed_radius_m,"
                << "protected_radius_m,"
                << "junction_overlap_valid,"
                << "junction_overlap_radius_m,"

                << "total_faces,"
                << "domain_faces,"
                << "obstacle_faces,"

                << "input_obstacle_count,"
                << "local_obstacle_count,"
                << "candidate_count,"
                << "generated_candidate_count,"
                << "active_witness_rounds,"
                << "witness_distance_tests,"
                << "obstacle_face_tests,"
                << "greedy_obstacle_face_count,"
                << "redundancy_removed,"
                << "safety_verified,"
                << "overlap_guaranteed,"

                << "construction_metric_valid,"
                << "construction_anisotropic_domain,"
                << "construction_utility_eig0,"
                << "construction_utility_eig1,"
                << "construction_utility_eig2,"
                << "construction_utility_anisotropy,"

                << "construction_extra_radius0_m,"
                << "construction_extra_radius1_m,"
                << "construction_extra_radius2_m,"
                << "mean_metric_damage,"
                << "min_metric_damage,"
                << "max_metric_damage,"

                << "construction_metric_source_piece_id,"
                << "construction_metric_mapping_distance,"

                << "reference_metric_valid,"
                << "reference_utility_eig0,"
                << "reference_utility_eig1,"
                << "reference_utility_eig2,"
                << "reference_metric_source_piece_id,"
                << "reference_metric_mapping_distance,"

                << "directional_reserve_valid,"
                << "hard_positive_m,"
                << "hard_negative_m,"
                << "hard_symmetric_m,"
                << "hard_span_m,"
                << "middle_positive_m,"
                << "middle_negative_m,"
                << "middle_symmetric_m,"
                << "middle_span_m,"
                << "easy_positive_m,"
                << "easy_negative_m,"
                << "easy_symmetric_m,"
                << "easy_span_m\n";
        }

        output <<
            std::setprecision(17);

        for (const auto &record :
             records)
        {
            output
                << 2 << ','

                << csv(record.case_id) << ','
                << csv(record.route_fingerprint) << ','
                << csv(record.method) << ','
                << csv(record.variant) << ','
                << record.repeat_id << ','
                << record.timestamp_s << ','
                << record.corridor_id << ','
                << record.source_segment_id << ','
                << record.geometry_mapping_valid << ','
                << csv(record.geometry_protocol) << ','
                << csv(record.construction_direction_basis) << ','
                << csv(record.reference_direction_source) << ','

                << record.seed_metric_valid << ','
                << record.seed_radius_m << ','
                << record.protected_radius_m << ','
                << record.junction_overlap_valid << ','
                << record.junction_overlap_radius_m << ','

                << record.total_faces << ','
                << record.domain_faces << ','
                << record.obstacle_faces << ','

                << record.input_obstacle_count << ','
                << record.local_obstacle_count << ','
                << record.candidate_count << ','
                << record.generated_candidate_count << ','
                << record.active_witness_rounds << ','
                << record.witness_distance_tests << ','
                << record.obstacle_face_tests << ','
                << record.greedy_obstacle_face_count << ','
                << record.redundancy_removed << ','
                << record.safety_verified << ','
                << record.overlap_guaranteed << ','

                << record.metric_valid << ','
                << record.anisotropic_domain << ','
                << record.utility_eig0 << ','
                << record.utility_eig1 << ','
                << record.utility_eig2 << ','
                << record.utility_anisotropy << ','
                << record.construction_extra_radius0_m << ','
                << record.construction_extra_radius1_m << ','
                << record.construction_extra_radius2_m << ','
                << record.mean_metric_damage << ','
                << record.min_metric_damage << ','
                << record.max_metric_damage << ','

                << record.metric_source_piece_id << ','
                << record.metric_mapping_distance << ','
                        
                << record.reference_metric_valid << ','
                << record.reference_utility_eig0 << ','
                << record.reference_utility_eig1 << ','
                << record.reference_utility_eig2 << ','
                << record.reference_metric_source_piece_id << ','
                << record.reference_metric_mapping_distance << ','
                        
                << record.directional_reserve_valid << ','
                << record.hard_positive_m << ','
                << record.hard_negative_m << ','
                << record.hard_symmetric_m << ','
                << record.hard_span_m << ','
                << record.middle_positive_m << ','
                << record.middle_negative_m << ','
                << record.middle_symmetric_m << ','
                << record.middle_span_m << ','
                << record.easy_positive_m << ','
                << record.easy_negative_m << ','
                << record.easy_symmetric_m << ','
                << record.easy_span_m
                << '\n';
        }

        return static_cast<bool>(
            output);
    }

    inline bool
    logControlledCorridorsV3(
        const std::vector<
            BenchmarkControlledCorridorRecordV3> &records)
    {
        if (!enabled_ ||
            records.empty())
        {
            return true;
        }
    
        std::lock_guard<std::mutex>
            lock(
                mutex_);
            
        if (!ensureDirectory())
        {
            return false;
        }
    
        const std::string path =
            directory_ +
            "/benchmark_corridors_v3.csv";
    
        const bool header =
            fileNeedsHeader(
                path);
            
        std::ofstream output(
            path,
            std::ios::out |
                std::ios::app);
        
        if (!output)
        {
            return false;
        }
    
    
        if (header)
        {
            output
                << "schema_version,"
        
                << "case_id,"
                << "route_fingerprint,"
                << "method,"
                << "variant,"
                << "repeat_id,"
                << "timestamp_s,"
                << "corridor_id,"
                << "source_segment_id,"
                << "geometry_mapping_valid,"
                << "geometry_protocol,"
        
                << "construction_algorithm,"
                << "construction_direction_basis,"
                << "reference_direction_source,"
                << "input_obstacle_count,"
                << "local_obstacle_count,"
                << "raw_face_count,"
                << "raw_domain_face_count,"
                << "raw_obstacle_face_count,"
                << "aw_candidate_count,"
                << "aw_active_rounds,"
                << "aw_witness_distance_tests,"
                << "aw_obstacle_face_tests,"
        
                << "common_safety_valid,"
                << "common_safe,"
                << "obstacle_surface_safe,"
                << "map_contained,"
                << "obstacle_sample_count,"
                << "worst_obstacle_index,"
                << "min_obstacle_exclusion_margin_m,"
                << "max_obstacle_penetration_m,"
                << "max_map_violation_m,"
        
                << "seed_metric_valid,"
                << "seed_radius_m,"
                << "protected_radius_prescribed,"
                << "prescribed_protected_radius_m,"
                << "prescribed_seed_satisfied,"
                << "junction_overlap_valid,"
                << "junction_overlap_radius_m,"
                << "prescribed_overlap_satisfied,"
        
                << "reference_metric_valid,"
                << "reference_utility_eig0,"
                << "reference_utility_eig1,"
                << "reference_utility_eig2,"
                << "reference_metric_source_piece_id,"
                << "reference_metric_mapping_distance,"
        
                << "point_width_valid,"
                << "point_reference_margin_m,"
                << "hard_positive_m,"
                << "hard_negative_m,"
                << "hard_width_m,"
                << "middle_positive_m,"
                << "middle_negative_m,"
                << "middle_width_m,"
                << "easy_positive_m,"
                << "easy_negative_m,"
                << "easy_width_m,"
        
                << "volume_valid,"
                << "volume_m3,"
                << "volume_vertex_count,"
                << "volume_triangle_count,"
                << "volume_max_vertex_violation_m,"
        
                << "effective_face_valid,"
                << "unique_plane_groups,"
                << "duplicate_rows,"
                << "effective_face_count,"
                << "redundant_plane_groups\n";
        }
    
    
        output <<
            std::setprecision(17);
    
    
        for (const auto &record :
             records)
        {
            output
                << 3 << ','
        
                << csv(record.case_id) << ','
                << csv(record.route_fingerprint) << ','
                << csv(record.method) << ','
                << csv(record.variant) << ','
                << record.repeat_id << ','
                << record.timestamp_s << ','
                << record.corridor_id << ','
                << record.source_segment_id << ','
                << record.geometry_mapping_valid << ','
                << csv(record.geometry_protocol) << ','
        
                << csv(record.construction_algorithm) << ','
                << csv(record.construction_direction_basis) << ','
                << csv(record.reference_direction_source) << ','
                << record.input_obstacle_count << ','
                << record.local_obstacle_count << ','
                << record.raw_face_count << ','
                << record.raw_domain_face_count << ','
                << record.raw_obstacle_face_count << ','
                << record.aw_candidate_count << ','
                << record.aw_active_rounds << ','
                << record.aw_witness_distance_tests << ','
                << record.aw_obstacle_face_tests << ','
        
                << record.common_safety_valid << ','
                << record.common_safe << ','
                << record.obstacle_surface_safe << ','
                << record.map_contained << ','
                << record.obstacle_sample_count << ','
                << record.worst_obstacle_index << ','
                << record.min_obstacle_exclusion_margin_m << ','
                << record.max_obstacle_penetration_m << ','
                << record.max_map_violation_m << ','
        
                << record.seed_metric_valid << ','
                << record.seed_radius_m << ','
                << record.protected_radius_prescribed << ','
                << record.prescribed_protected_radius_m << ','
                << record.prescribed_seed_satisfied << ','
                << record.junction_overlap_valid << ','
                << record.junction_overlap_radius_m << ','
                << record.prescribed_overlap_satisfied << ','
        
                << record.reference_metric_valid << ','
                << record.reference_utility_eig0 << ','
                << record.reference_utility_eig1 << ','
                << record.reference_utility_eig2 << ','
                << record.reference_metric_source_piece_id << ','
                << record.reference_metric_mapping_distance << ','
        
                << record.point_width_valid << ','
                << record.point_reference_margin_m << ','
                << record.hard_positive_m << ','
                << record.hard_negative_m << ','
                << record.hard_width_m << ','
                << record.middle_positive_m << ','
                << record.middle_negative_m << ','
                << record.middle_width_m << ','
                << record.easy_positive_m << ','
                << record.easy_negative_m << ','
                << record.easy_width_m << ','
        
                << record.volume_valid << ','
                << record.volume_m3 << ','
                << record.volume_vertex_count << ','
                << record.volume_triangle_count << ','
                << record.volume_max_vertex_violation_m << ','
        
                << record.effective_face_valid << ','
                << record.unique_plane_groups << ','
                << record.duplicate_rows << ','
                << record.effective_face_count << ','
                << record.redundant_plane_groups
        
                << '\n';
        }
    
    
        return static_cast<bool>(
            output);
    }
};

} // namespace gcopter_benchmark

#endif