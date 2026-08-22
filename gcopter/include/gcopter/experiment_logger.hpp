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
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace gcopter_experiment
{

    struct RunRecord
    {
        std::string run_id;
        std::string experiment_tag;
        std::string method = "gcopter_firi";
        std::string status;
        double timestamp_s = 0.0;
        bool success = false;
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
        int trajectory_piece_count = 0;
        double trajectory_duration_s = 0.0;
        double trajectory_length_m = 0.0;
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

        inline bool log(const RunRecord &record)
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

            const std::string path = directory_ + "/gcopter_runs.csv";
            const bool header = fileNeedsHeader(path);
            std::ofstream output(path, std::ios::out | std::ios::app);
            if (!output)
            {
                return false;
            }
            if (header)
            {
                output << "schema_version,run_id,timestamp_s,experiment_tag,method,status,success,"
                          "map_point_count,route_point_count,corridor_count,total_faces,mean_faces,"
                          "path_search_ms,corridor_generation_ms,optimizer_setup_ms,optimizer_ms,"
                          "total_planning_ms,final_cost,trajectory_piece_count,trajectory_duration_s,"
                          "trajectory_length_m\n";
            }
            output << std::setprecision(17)
                   << 1 << ',' << csv(record.run_id) << ',' << record.timestamp_s << ','
                   << csv(record.experiment_tag) << ',' << csv(record.method) << ','
                   << csv(record.status) << ',' << record.success << ','
                   << record.map_point_count << ',' << record.route_point_count << ','
                   << record.corridor_count << ',' << record.total_faces << ','
                   << record.mean_faces << ',' << record.path_search_ms << ','
                   << record.corridor_generation_ms << ',' << record.optimizer_setup_ms << ','
                   << record.optimizer_ms << ',' << record.total_planning_ms << ','
                   << record.final_cost << ',' << record.trajectory_piece_count << ','
                   << record.trajectory_duration_s << ',' << record.trajectory_length_m << '\n';
            return static_cast<bool>(output);
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
