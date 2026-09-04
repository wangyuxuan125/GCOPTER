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
            "/benchmark_runs_v1.csv";

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

            << 1 << ','

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
};

} // namespace gcopter_benchmark

#endif