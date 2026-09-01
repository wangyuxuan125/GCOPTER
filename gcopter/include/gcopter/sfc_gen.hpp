/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"
#include "traj_relevant_corridor.hpp"

#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <deque>
#include <cstdint>
#include <memory>
#include <Eigen/Eigen>

namespace sfc_gen
{

    struct TrajectoryFavorableFiriInfo
    {
        int face_count = 0;
        bool face_budget_saturated = false;
        int unresolved_constraint_count = 0;
        int unresolved_boundary_count = 0;
        int unresolved_obstacle_count = 0;
        bool budget_exchange_attempted = false;
        bool budget_exchange_accepted = false;
        double directional_radius = 0.0;
        Eigen::Vector3d direction =
            Eigen::Vector3d::Zero();

        int obstacle_face_count =
            0;

        double mean_metric_damage =
            0.0;

        double min_metric_damage =
            0.0;

        double max_metric_damage =
            0.0;
    };

    struct SegmentDeformationMetric
    {
        int source_piece_id =
            -1;

        double mapping_distance =
            0.0;

        Eigen::Matrix3d utility =
            Eigen::Matrix3d::Identity();

        bool valid =
            false;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using SegmentDeformationMetrics =
        std::vector<
            SegmentDeformationMetric,
            Eigen::aligned_allocator<
                SegmentDeformationMetric>>;

    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p,
                           const std::uint_fast32_t routeSeed = 0)
    {
        if (routeSeed != 0)
        {
            ompl::RNG::setSeed(routeSeed);
        }
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);

        auto si(std::make_shared<ompl::base::SpaceInformation>(space));

        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        si->setup();

        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] = s(2) - lb(2);
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        goal[2] = g(2) - lb(2);

        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();

        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;
        if (solved)
        {
            p.clear();
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
            }
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }

        return cost;
    }

    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
    }

    // Trajectory-conditioned FIRI.  It deliberately preserves the same path
    // segmentation, local obstacle crop, boundary and overlap construction as
    // convexCover(); only the FIRI ellipsoid objective and its hard face budget
    // are changed.  This makes firi and tf_firi suitable paired baselines.
    inline bool trajectoryFavorableConvexCover(
        const std::vector<Eigen::Vector3d> &path,
        const std::vector<Eigen::Vector3d> &points,
        const Eigen::Vector3d &lowCorner,
        const Eigen::Vector3d &highCorner,
        const double &progress,
        const double &range,
        const firi::TrajectoryFavorableOptions &options,
        std::vector<Eigen::MatrixX4d> &hpolys,
        std::vector<TrajectoryFavorableFiriInfo> &infos,
        const SegmentDeformationMetrics *segmentMetrics =
            nullptr,
        const double eps = 1.0e-6)
    {
        hpolys.clear();
        infos.clear();
        if (path.size() < 2)
        {
            return false;
        }

        const int n = static_cast<int>(path.size());
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path.front();
        int rawSegmentId = 0;
        std::vector<Eigen::Vector3d> validPc;
        validPc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i++];
            }

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            validPc.clear();
            for (const Eigen::Vector3d &point : points)
            {
                if ((bd.leftCols<3>() * point + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    validPc.emplace_back(point);
                }
            }
            Eigen::Matrix3Xd pc(3, static_cast<int>(validPc.size()));
            for (int pointId = 0; pointId < static_cast<int>(validPc.size()); ++pointId)
            {
                pc.col(pointId) = validPc[pointId];
            }

            firi::TrajectoryFavorableOptions segmentOptions = options;
            segmentOptions.enabled = true;
            segmentOptions.direction = b - a;
                    
            segmentOptions.metric_enabled =
                false;

            segmentOptions.deformation_utility =
                Eigen::Matrix3d::Identity();

            if (segmentMetrics != nullptr)
            {
                if (rawSegmentId >=
                    static_cast<int>(
                        segmentMetrics->size()))
                {
                    return false;
                }
            
                const auto &mappedMetric =
                    (*segmentMetrics)[rawSegmentId];
            
                if (mappedMetric.valid)
                {
                    segmentOptions.metric_enabled =
                        true;
                
                    segmentOptions.deformation_utility =
                        mappedMetric.utility;
                }
            }
            firi::TrajectoryFavorableDiagnostics diagnostics;
            if (!firi::firi(bd, pc, a, b, hp, 4, eps,
                            segmentOptions, &diagnostics))
            {
                infos.push_back({diagnostics.face_count,
                                 diagnostics.face_budget_saturated,
                                 diagnostics.unresolved_constraint_count,
                                 diagnostics.unresolved_boundary_count,
                                 diagnostics.unresolved_obstacle_count,
                                 diagnostics.budget_exchange_attempted,
                                 diagnostics.budget_exchange_accepted,
                                 diagnostics.directional_radius,
                                 segmentOptions.direction.normalized(),
                                 diagnostics.obstacle_face_count,
                                 diagnostics.mean_metric_damage,
                                 diagnostics.min_metric_damage,
                                 diagnostics.max_metric_damage});
                return false;
            }

            if (!hpolys.empty())
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::TrajectoryFavorableOptions gapOptions = segmentOptions;
                    gapOptions.direction = infos.back().direction;
                    firi::TrajectoryFavorableDiagnostics gapDiagnostics;
                    if (!firi::firi(bd, pc, a, a, gap, 1, eps,
                                    gapOptions, &gapDiagnostics))
                    {
                        infos.push_back({gapDiagnostics.face_count,
                                         gapDiagnostics.face_budget_saturated,
                                         gapDiagnostics.unresolved_constraint_count,
                                         gapDiagnostics.unresolved_boundary_count,
                                         gapDiagnostics.unresolved_obstacle_count,
                                         gapDiagnostics.budget_exchange_attempted,
                                         gapDiagnostics.budget_exchange_accepted,
                                         gapDiagnostics.directional_radius,
                                         gapOptions.direction.normalized(),
                                         gapDiagnostics.obstacle_face_count,
                                         gapDiagnostics.mean_metric_damage,
                                         gapDiagnostics.min_metric_damage,
                                         gapDiagnostics.max_metric_damage});
                        return false;
                    }
                    hpolys.emplace_back(gap);
                    infos.push_back({gapDiagnostics.face_count,
                                     gapDiagnostics.face_budget_saturated,
                                     gapDiagnostics.unresolved_constraint_count,
                                     gapDiagnostics.unresolved_boundary_count,
                                     gapDiagnostics.unresolved_obstacle_count,
                                     gapDiagnostics.budget_exchange_attempted,
                                     gapDiagnostics.budget_exchange_accepted,
                                     gapDiagnostics.directional_radius,
                                     gapOptions.direction.normalized(),
                                     gapDiagnostics.obstacle_face_count,
                                     gapDiagnostics.mean_metric_damage,
                                     gapDiagnostics.min_metric_damage,
                                     gapDiagnostics.max_metric_damage});
                }
            }

            hpolys.emplace_back(hp);
            infos.push_back({diagnostics.face_count,
                             diagnostics.face_budget_saturated,
                             diagnostics.unresolved_constraint_count,
                             diagnostics.unresolved_boundary_count,
                             diagnostics.unresolved_obstacle_count,
                             diagnostics.budget_exchange_attempted,
                             diagnostics.budget_exchange_accepted,
                             diagnostics.directional_radius,
                             segmentOptions.direction.normalized(),
                             diagnostics.obstacle_face_count,
                             diagnostics.mean_metric_damage,
                             diagnostics.min_metric_damage,
                             diagnostics.max_metric_damage});
            ++rawSegmentId;
        }
        if (hpolys.empty())
        {
            return false;
        }

        if (segmentMetrics != nullptr &&
            rawSegmentId != static_cast<int>(segmentMetrics->size()))
        {
            return false;
        }

        // Apply the same overlap shortcut as the native FIRI baseline while
        // retaining the diagnostics for every selected polytope.
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        std::vector<TrajectoryFavorableFiriInfo> itemp = infos;
        if (htemp.size() == 1)
        {
            htemp.insert(htemp.begin(), htemp.front());
            itemp.insert(itemp.begin(), itemp.front());
        }
        std::deque<int> indices;
        indices.push_front(static_cast<int>(htemp.size()) - 1);
        for (int i = static_cast<int>(htemp.size()) - 1; i >= 0; --i)
        {
            for (int j = 0; j < i; ++j)
            {
                const bool overlap = j < i - 1
                                         ? geo_utils::overlap(htemp[i], htemp[j], 0.01)
                                         : true;
                if (overlap)
                {
                    indices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        hpolys.clear();
        infos.clear();
        for (const int index : indices)
        {
            hpolys.push_back(htemp[index]);
            infos.push_back(itemp[index]);
        }
        return true;
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
