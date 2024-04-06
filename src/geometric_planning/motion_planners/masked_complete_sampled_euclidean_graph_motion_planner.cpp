/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "grstapse/geometric_planning/motion_planners/masked_complete_sampled_euclidean_graph_motion_planner.hpp"

// External
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/map.hpp>
// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"

namespace grstapse
{
    MaskedCompleteSampledEuclideanGraphMotionPlanner::MaskedCompleteSampledEuclideanGraphMotionPlanner(
        const std::shared_ptr<const ParametersBase>& parameters,
        const std::shared_ptr<SampledEuclideanGraphEnvironment>& environment)
        : CompleteSampledEuclideanGraphMotionPlanner(parameters, environment)
    {
        // Default mask is all true
        for(unsigned int i: ::ranges::view::iota(0u, environment->numGraphs()))
        {
            m_indices.emplace(i, i);
        }
    }

    void MaskedCompleteSampledEuclideanGraphMotionPlanner::setMask(const std::vector<bool>& mask)
    {
        m_indices.clear();
        for(auto [i, j]: ::ranges::view::enumerate(::ranges::view::keys(
                ::ranges::view::enumerate(mask) | ::ranges::view::filter(
                                                      [](const std::pair<unsigned int, bool>& p) -> bool
                                                      {
                                                          return p.second;
                                                      }))))
        {
            m_indices.emplace(i, j);
        }
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> MaskedCompleteSampledEuclideanGraphMotionPlanner::query(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration)
    {
        if(index >= m_indices.size())
        {
            throw createLogicError("Index out of bounds of mask");
        }
        return SampledEuclideanGraphMotionPlannerBase::query(m_indices.at(index),
                                                             species,
                                                             initial_configuration,
                                                             goal_configuration);
    }
    bool MaskedCompleteSampledEuclideanGraphMotionPlanner::isMemoized(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration) const
    {
        if(index >= m_indices.size())
        {
            throw createLogicError("Index out of bounds of mask");
        }
        return SampledEuclideanGraphMotionPlannerBase::isMemoized(m_indices.at(index),
                                                                  species,
                                                                  initial_configuration,
                                                                  goal_configuration);
    }
    float MaskedCompleteSampledEuclideanGraphMotionPlanner::durationQuery(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration)
    {
        if(index >= m_indices.size())
        {
            throw createLogicError("Index out of bounds of mask");
        }
        return SampledEuclideanGraphMotionPlannerBase::durationQuery(m_indices.at(index),
                                                                     species,
                                                                     initial_configuration,
                                                                     goal_configuration);
    }

}  // namespace grstapse