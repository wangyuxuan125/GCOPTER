#ifndef GCOPTER_BENCHMARK_LOGGER_HPP
#define GCOPTER_BENCHMARK_LOGGER_HPP

#include <cerrno>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

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

} // namespace gcopter_benchmark

#endif