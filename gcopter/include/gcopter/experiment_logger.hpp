#ifndef GCOPTER_EXPERIMENT_LOGGER_HPP
#define GCOPTER_EXPERIMENT_LOGGER_HPP

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace gcopter_experiment
{

    struct RunRecord
    {
        std::string run_id;
        std::string experiment_tag;
        std::string requested_method = "firi";
        std::string method = "firi";
        bool fallback_used = false;
        std::string status;
        double timestamp_s = 0.0;
        bool success = false;
        int map_seed = 0;
        double start_x = 0.0;
        double start_y = 0.0;
        double start_z = 0.0;
        double goal_x = 0.0;
        double goal_y = 0.0;
        double goal_z = 0.0;
        double voxel_width_m = 0.0;
        double dilate_radius_m = 0.0;
        double route_timeout_s = 0.0;
        double max_velocity_mps = 0.0;
        double max_body_rate_radps = 0.0;
        double max_tilt_rad = 0.0;
        double min_thrust = 0.0;
        double max_thrust = 0.0;
        int map_point_count = 0;
        int route_point_count = 0;
        int corridor_count = 0;
        int total_faces = 0;
        double mean_faces = 0.0;
        double path_search_ms = 0.0;
        double corridor_generation_ms = 0.0;
        double optimizer_setup_ms = 0.0;
        double optimizer_ms = 0.0;
        double total_planning_ms = 0.0;
        double final_cost = 0.0;
        int corridor_constrained_piece_count = 0;
        double corridor_penalty_cost_initial = 0.0;
        double corridor_penalty_cost_final = 0.0;
        double max_corridor_violation_initial_m = 0.0;
        double max_corridor_violation_final_m = 0.0;
        int trajectory_piece_count = 0;
        double trajectory_duration_s = 0.0;
        double trajectory_length_m = 0.0;
    };

    struct CorridorRecord
    {
        int piece_id = -1;
        int face_count = 0;
        int obstacle_face_count = 0;
        int obstacle_point_count = 0;
        bool face_budget_saturated = false;
        double generation_time_ms = 0.0;
        double weighted_width = 0.0;
        double min_sample_slack = 0.0;
        double anchor_clearance_radius = 0.0;
        double directional_radius_m = 0.0;
        double directional_width_weight = 0.0;
        double face_count_weight = 0.0;
        double overlap_radius_to_next = -1.0;
        bool valid = false;
        bool direction_fallback = false;
        std::string failure_reason = "none";
    };

    class CsvLogger
    {
    public:
        CsvLogger(const bool enabled,
                  const std::string &directory,
                  const std::string &experimentTag)
            : enabled_(enabled),
              directory_(directory),
              experimentTag_(experimentTag.empty() ? "default" : experimentTag),
              sequence_(0)
        {
            while (directory_.size() > 1 && directory_.back() == '/')
            {
                directory_.pop_back();
            }
        }

        inline bool enabled() const
        {
            return enabled_;
        }

        inline const std::string &experimentTag() const
        {
            return experimentTag_;
        }

        inline std::string makeRunId()
        {
            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
            std::lock_guard<std::mutex> lock(mutex_);
            std::ostringstream stream;
            stream << micros << "-p" << static_cast<long>(::getpid()) << '-' << sequence_++;
            return stream.str();
        }

        inline bool log(const RunRecord &record,
                        const std::vector<CorridorRecord> &corridors = {})
        {
            if (!enabled_)
            {
                return true;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if (!ensureDirectory())
            {
                return false;
            }

            const std::string path = directory_ + "/gcopter_runs_v7.csv";
            const bool header = fileNeedsHeader(path);
            std::ofstream output(path, std::ios::out | std::ios::app);
            if (!output)
            {
                return false;
            }
            if (header)
            {
                output << "schema_version,run_id,timestamp_s,experiment_tag,requested_method,method,"
                          "fallback_used,status,success,"
                          "map_seed,start_x,start_y,start_z,goal_x,goal_y,goal_z,"
                          "voxel_width_m,dilate_radius_m,route_timeout_s,max_velocity_mps,"
                          "max_body_rate_radps,max_tilt_rad,min_thrust,max_thrust,"
                          "map_point_count,route_point_count,corridor_count,total_faces,mean_faces,"
                          "path_search_ms,corridor_generation_ms,optimizer_setup_ms,optimizer_ms,"
                          "total_planning_ms,final_cost,trajectory_piece_count,trajectory_duration_s,"
                          "trajectory_length_m,corridor_constrained_piece_count,"
                          "corridor_penalty_cost_initial,corridor_penalty_cost_final,"
                          "max_corridor_violation_initial_m,max_corridor_violation_final_m\n";
            }
            output << std::setprecision(17)
                   << 7 << ',' << csv(record.run_id) << ',' << record.timestamp_s << ','
                   << csv(record.experiment_tag) << ',' << csv(record.requested_method) << ','
                   << csv(record.method) << ',' << record.fallback_used << ','
                   << csv(record.status) << ',' << record.success << ','
                   << record.map_seed << ',' << record.start_x << ',' << record.start_y << ','
                   << record.start_z << ',' << record.goal_x << ',' << record.goal_y << ','
                   << record.goal_z << ',' << record.voxel_width_m << ','
                   << record.dilate_radius_m << ',' << record.route_timeout_s << ','
                   << record.max_velocity_mps << ',' << record.max_body_rate_radps << ','
                   << record.max_tilt_rad << ',' << record.min_thrust << ','
                   << record.max_thrust << ','
                   << record.map_point_count << ',' << record.route_point_count << ','
                   << record.corridor_count << ',' << record.total_faces << ','
                   << record.mean_faces << ',' << record.path_search_ms << ','
                   << record.corridor_generation_ms << ',' << record.optimizer_setup_ms << ','
                   << record.optimizer_ms << ',' << record.total_planning_ms << ','
                   << record.final_cost << ',' << record.trajectory_piece_count << ','
                   << record.trajectory_duration_s << ',' << record.trajectory_length_m << ','
                   << record.corridor_constrained_piece_count << ','
                   << record.corridor_penalty_cost_initial << ','
                   << record.corridor_penalty_cost_final << ','
                   << record.max_corridor_violation_initial_m << ','
                   << record.max_corridor_violation_final_m << '\n';
            if (!output)
            {
                return false;
            }
            if (corridors.empty())
            {
                return true;
            }

            const std::string corridorPath = directory_ + "/gcopter_corridors_v7.csv";
            const bool corridorHeader = fileNeedsHeader(corridorPath);
            std::ofstream corridorOutput(corridorPath, std::ios::out | std::ios::app);
            if (!corridorOutput)
            {
                return false;
            }
            if (corridorHeader)
            {
                corridorOutput << "schema_version,run_id,timestamp_s,experiment_tag,requested_method,"
                                  "method,piece_id,face_count,obstacle_face_count,obstacle_point_count,"
                                  "face_budget_saturated,generation_time_ms,weighted_width,"
                                  "min_sample_slack,anchor_clearance_radius,directional_radius_m,"
                                  "directional_width_weight,face_count_weight,overlap_radius_to_next,"
                                  "valid,direction_fallback,failure_reason\n";
            }
            corridorOutput << std::setprecision(17);
            for (const CorridorRecord &corridor : corridors)
            {
                corridorOutput << 7 << ',' << csv(record.run_id) << ',' << record.timestamp_s << ','
                               << csv(record.experiment_tag) << ',' << csv(record.requested_method) << ','
                               << csv(record.method) << ',' << corridor.piece_id << ','
                               << corridor.face_count << ',' << corridor.obstacle_face_count << ','
                               << corridor.obstacle_point_count << ','
                               << corridor.face_budget_saturated << ','
                               << corridor.generation_time_ms << ','
                               << corridor.weighted_width << ',' << corridor.min_sample_slack << ','
                               << corridor.anchor_clearance_radius << ','
                               << corridor.directional_radius_m << ','
                               << corridor.directional_width_weight << ','
                               << corridor.face_count_weight << ','
                               << corridor.overlap_radius_to_next << ',' << corridor.valid << ','
                               << corridor.direction_fallback << ',' << csv(corridor.failure_reason) << '\n';
            }
            return static_cast<bool>(corridorOutput);
        }

    private:
        inline bool ensureDirectory() const
        {
            if (directory_.empty())
            {
                return false;
            }
            std::string current = directory_.front() == '/' ? "/" : "";
            std::istringstream path(directory_);
            std::string part;
            while (std::getline(path, part, '/'))
            {
                if (part.empty())
                {
                    continue;
                }
                if (!current.empty() && current.back() != '/')
                {
                    current += '/';
                }
                current += part;
                if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
                {
                    return false;
                }
            }
            return true;
        }

        static inline bool fileNeedsHeader(const std::string &path)
        {
            std::ifstream input(path, std::ios::binary);
            return !input || input.peek() == std::ifstream::traits_type::eof();
        }

        static inline std::string csv(const std::string &value)
        {
            if (value.find_first_of(",\"\n\r") == std::string::npos)
            {
                return value;
            }
            std::string escaped = "\"";
            for (const char c : value)
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

        bool enabled_;
        std::string directory_;
        std::string experimentTag_;
        std::uint64_t sequence_;
        std::mutex mutex_;
    };

} // namespace gcopter_experiment

#endif
