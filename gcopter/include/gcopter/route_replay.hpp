#ifndef GCOPTER_ROUTE_REPLAY_HPP
#define GCOPTER_ROUTE_REPLAY_HPP

#include <Eigen/Eigen>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

namespace gcopter_benchmark
{

struct RouteValidationResult
{
    bool valid =
        false;

    int point_count =
        0;

    int segment_count =
        0;

    int checked_samples =
        0;

    int occupied_samples =
        0;

    int first_bad_segment =
        -1;

    double route_length_m =
        0.0;

    double max_segment_length_m =
        0.0;

    double validation_ms =
        0.0;
};


inline double routeLength(
    const std::vector<Eigen::Vector3d> &route)
{
    double length =
        0.0;

    for (std::size_t i = 1;
         i < route.size();
         ++i)
    {
        length +=
            (route[i] -
             route[i - 1])
                .norm();
    }

    return length;
}


inline std::string routeFingerprint(
    const std::vector<Eigen::Vector3d> &route)
{
    // ------------------------------------------------------------
    // FNV-1a over a canonical decimal representation.
    //
    // setprecision(17) guarantees round-trip representation for
    // IEEE-754 double on the same benchmark infrastructure.
    // ------------------------------------------------------------
    std::ostringstream canonical;

    canonical
        << std::setprecision(17);

    canonical
        << route.size()
        << ';';

    for (const auto &point :
         route)
    {
        canonical
            << point.x()
            << ','
            << point.y()
            << ','
            << point.z()
            << ';';
    }

    const std::string text =
        canonical.str();

    std::uint64_t hash =
        1469598103934665603ULL;

    constexpr std::uint64_t
        prime =
            1099511628211ULL;

    for (const unsigned char c :
         text)
    {
        hash ^=
            static_cast<std::uint64_t>(
                c);

        hash *=
            prime;
    }

    std::ostringstream result;

    result
        << std::hex
        << std::setw(16)
        << std::setfill('0')
        << hash;

    return result.str();
}


inline bool ensureDirectory(
    const std::string &directory)
{
    if (directory.empty())
    {
        return true;
    }

    std::string current =
        directory.front() == '/'
            ? "/"
            : "";

    std::istringstream pathStream(
        directory);

    std::string part;

    while (std::getline(
        pathStream,
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


inline bool ensureParentDirectory(
    const std::string &filePath)
{
    const std::size_t slash =
        filePath.find_last_of('/');

    if (slash ==
        std::string::npos)
    {
        return true;
    }

    const std::string parent =
        filePath.substr(
            0,
            slash);

    return ensureDirectory(
        parent);
}


inline bool saveRoute(
    const std::string &filePath,
    const std::vector<Eigen::Vector3d> &route)
{
    if (filePath.empty() ||
        route.size() < 2)
    {
        return false;
    }

    for (const auto &point :
         route)
    {
        if (!point.allFinite())
        {
            return false;
        }
    }

    if (!ensureParentDirectory(
            filePath))
    {
        return false;
    }

    std::ofstream output(
        filePath,
        std::ios::out |
            std::ios::trunc);

    if (!output)
    {
        return false;
    }

    output
        << "# GCOPTER_ROUTE_V1\n";

    output
        << "# point_count "
        << route.size()
        << '\n';

    output
        << std::setprecision(17);

    for (const auto &point :
         route)
    {
        output
            << point.x()
            << ' '
            << point.y()
            << ' '
            << point.z()
            << '\n';
    }

    return static_cast<bool>(
        output);
}


inline bool loadRoute(
    const std::string &filePath,
    std::vector<Eigen::Vector3d> &route)
{
    route.clear();

    if (filePath.empty())
    {
        return false;
    }

    std::ifstream input(
        filePath);

    if (!input)
    {
        return false;
    }

    std::string line;

    while (std::getline(
        input,
        line))
    {
        if (line.empty())
        {
            continue;
        }

        const std::size_t first =
            line.find_first_not_of(
                " \t\r\n");

        if (first ==
            std::string::npos)
        {
            continue;
        }

        if (line[first] == '#')
        {
            continue;
        }

        // Also accept comma-separated coordinates.
        std::replace(
            line.begin(),
            line.end(),
            ',',
            ' ');

        std::istringstream
            lineStream(
                line);

        double x =
            0.0;

        double y =
            0.0;

        double z =
            0.0;

        if (!(lineStream >>
              x >>
              y >>
              z))
        {
            route.clear();

            return false;
        }

        if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z))
        {
            route.clear();

            return false;
        }

        route.emplace_back(
            x,
            y,
            z);
    }

    return route.size() >= 2;
}


template <typename Map>
inline RouteValidationResult
validateRoute(
    const std::vector<Eigen::Vector3d> &route,
    const Map &map,
    const double sampleStepM)
{
    RouteValidationResult result;

    const auto started =
        std::chrono::
            steady_clock::now();

    result.point_count =
        static_cast<int>(
            route.size());

    result.segment_count =
        std::max(
            result.point_count - 1,
            0);

    if (route.size() < 2 ||
        !std::isfinite(
            sampleStepM) ||
        sampleStepM <= 0.0)
    {
        return result;
    }

    result.valid =
        true;

    for (int segmentId = 0;
         segmentId <
             result.segment_count;
         ++segmentId)
    {
        const Eigen::Vector3d &a =
            route[
                segmentId];

        const Eigen::Vector3d &b =
            route[
                segmentId + 1];

        if (!a.allFinite() ||
            !b.allFinite())
        {
            result.valid =
                false;

            result.first_bad_segment =
                segmentId;

            break;
        }

        const double length =
            (b - a).norm();

        result.route_length_m +=
            length;

        result.max_segment_length_m =
            std::max(
                result
                    .max_segment_length_m,
                length);

        const int steps =
            std::max(
                1,
                static_cast<int>(
                    std::ceil(
                        length /
                        sampleStepM)));

        for (int sampleId = 0;
             sampleId <= steps;
             ++sampleId)
        {
            const double alpha =
                static_cast<double>(
                    sampleId) /
                static_cast<double>(
                    steps);

            const Eigen::Vector3d
                point =
                    (1.0 - alpha) *
                        a +
                    alpha *
                        b;

            ++result
                  .checked_samples;

            if (map.query(
                    point) != 0)
            {
                ++result
                      .occupied_samples;

                result.valid =
                    false;

                if (result
                        .first_bad_segment <
                    0)
                {
                    result
                        .first_bad_segment =
                            segmentId;
                }
            }
        }
    }

    result.validation_ms =
        std::chrono::duration<
            double,
            std::milli>(
                std::chrono::
                    steady_clock::now() -
                started)
            .count();

    return result;
}

} // namespace gcopter_benchmark

#endif