#ifndef DAC_SFC_VIDEO_TRACE_HPP
#define DAC_SFC_VIDEO_TRACE_HPP

#include "gcopter/traj_relevant_corridor.hpp"
#include "gcopter/trajectory.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace dac_sfc_video
{
inline void writeVec(std::ostream &out, const Eigen::Vector3d &v)
{
    out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}

inline void writePlane(std::ostream &out, const Eigen::Vector4d &v)
{
    out << '[' << v(0) << ',' << v(1) << ',' << v(2) << ',' << v(3) << ']';
}

inline void writePoints(std::ostream &out,
                        const std::vector<Eigen::Vector3d> &points)
{
    out << '[';
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (i) out << ',';
        writeVec(out, points[i]);
    }
    out << ']';
}

template <typename Allocator>
inline void writeAlignedPoints(
    std::ostream &out,
    const std::vector<Eigen::Vector3d, Allocator> &points)
{
    out << '[';
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (i) out << ',';
        writeVec(out, points[i]);
    }
    out << ']';
}

inline void writePoly(std::ostream &out, const Eigen::MatrixX4d &poly)
{
    out << '[';
    for (int i = 0; i < poly.rows(); ++i)
    {
        if (i) out << ',';
        const Eigen::Vector4d plane = poly.row(i).transpose();
        writePlane(out, plane);
    }
    out << ']';
}

template <int D>
inline void writeTrajectory(std::ostream &out, const Trajectory<D> &traj)
{
    out << '[';
    if (traj.getPieceNum() > 0)
    {
        const double duration = traj.getTotalDuration();
        const int intervals = std::min(1500, std::max(1,
            static_cast<int>(duration / 0.08) + 1));
        for (int i = 0; i <= intervals; ++i)
        {
            if (i) out << ',';
            writeVec(out, traj.getPos(duration * i / intervals));
        }
    }
    out << ']';
}

// Emit only after the planning and benchmark records have been finalized.
// A temporary file prevents a partial recording from looking valid.
inline bool write(const std::string &path,
                  const int mapSeed, const int routeSeed,
                  const int segmentId,
                  const std::vector<Eigen::Vector3d> &route,
                  const Trajectory<5> &probe,
                  const Trajectory<5> &displayedTrajectory,
                  const std::string &displayedMethod,
                  const traj_relevant::CompactCorridorTrace &trace,
                  const std::vector<Eigen::MatrixX4d> &corridors)
{
    if (!trace.success || path.empty()) return false;
    const std::string temporary = path + ".tmp";
    std::ofstream out(temporary.c_str(), std::ios::trunc);
    if (!out) return false;
    out << std::setprecision(17);
    out << "{\"schema\":1,\"frame\":\"odom\",\"map_seed\":"
        << mapSeed << ",\"route_seed\":" << routeSeed
        << ",\"segment_id\":" << segmentId
        << ",\"trajectory_method\":\"" << displayedMethod << "\"";
    out << ",\"route\":"; writePoints(out, route);
    out << ",\"probe\":"; writeTrajectory(out, probe);
    out << ",\"trajectory\":"; writeTrajectory(out, displayedTrajectory);
    out << ",\"a\":"; writeVec(out, trace.a);
    out << ",\"b\":"; writeVec(out, trace.b);
    out << ",\"eigenvalues_ascending\":"; writeVec(out, trace.eigenvalues);
    out << ",\"eigenvectors_columns_ascending\":[";
    for (int i = 0; i < 3; ++i)
    {
        if (i) out << ',';
        const Eigen::Vector3d direction = trace.eigenvectors.col(i);
        writeVec(out, direction);
    }
    out << "],\"extra_radii_ascending\":";
    writeVec(out, trace.extra_radii);
    out << ",\"half_widths_ascending\":";
    writeVec(out, trace.half_widths);
    out << ",\"metric_valid\":" << (trace.metric_valid ? "true" : "false")
        << ",\"anisotropic_domain\":"
        << (trace.anisotropic_domain ? "true" : "false")
        << ",\"overlap_radius\":" << trace.overlap_radius;
    out << ",\"domain_planes\":"; writePoly(out, trace.domain_planes);
    out << ",\"local_obstacles\":";
    writeAlignedPoints(out, trace.local_obstacles);
    out << ",\"steps\":[";
    for (size_t i = 0; i < trace.steps.size(); ++i)
    {
        if (i) out << ',';
        const auto &step = trace.steps[i];
        out << "{\"witness\":"; writeVec(out, step.witness);
        out << ",\"projection\":"; writeVec(out, step.projection);
        out << ",\"plane\":"; writePlane(out, step.plane);
        out << ",\"excluded_obstacle_ids\":[";
        for (size_t j = 0; j < step.excluded_obstacle_ids.size(); ++j)
        {
            if (j) out << ',';
            out << step.excluded_obstacle_ids[j];
        }
        out << "]}";
    }
    out << "],\"candidate_planes\":[";
    for (size_t i = 0; i < trace.candidate_planes.size(); ++i)
    {
        if (i) out << ',';
        writePlane(out, trace.candidate_planes[i]);
    }
    out << "],\"removed_candidate_ids\":[";
    for (size_t i = 0; i < trace.removed_candidate_ids.size(); ++i)
    {
        if (i) out << ',';
        out << trace.removed_candidate_ids[i];
    }
    out << "],\"selected_raw_polytope\":";
    writePoly(out, trace.final_polytope);
    out << ",\"retained_corridors\":[";
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        if (i) out << ',';
        writePoly(out, corridors[i]);
    }
    out << "]}\n";
    out.close();
    if (!out || std::rename(temporary.c_str(), path.c_str()) != 0)
    {
        std::remove(temporary.c_str());
        return false;
    }
    return true;
}
} // namespace dac_sfc_video

#endif
