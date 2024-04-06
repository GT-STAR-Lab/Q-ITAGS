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
#include "grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner_base.hpp"

// Local
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/singular_euclidean_graph_motion_planner_base.hpp"

namespace grstapse
{
    SampledEuclideanGraphMotionPlannerBase::SampledEuclideanGraphMotionPlannerBase(
        const std::shared_ptr<const ParametersBase>& parameters,
        const std::shared_ptr<SampledEuclideanGraphEnvironment>& environment)
        : EuclideanGraphMotionPlannerBase(parameters, environment)
    {}

    std::shared_ptr<const MotionPlannerQueryResultBase> SampledEuclideanGraphMotionPlannerBase::query(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration)
    {
        assert(index < m_sub_motion_planners.size());
        return m_sub_motion_planners[index]->query(species, initial_configuration, goal_configuration);
    }

    bool SampledEuclideanGraphMotionPlannerBase::isMemoized(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration) const
    {
        assert(index < m_sub_motion_planners.size());
        return m_sub_motion_planners[index]->isMemoized(species, initial_configuration, goal_configuration);
    }

    float SampledEuclideanGraphMotionPlannerBase::durationQuery(
        unsigned int index,
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration)
    {
        assert(index < m_sub_motion_planners.size());
        return m_sub_motion_planners[index]->durationQuery(species, initial_configuration, goal_configuration);
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> SampledEuclideanGraphMotionPlannerBase::computeMotionPlan(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        // This is intentional
        throw std::logic_error("Not implemented");
    }
}  // namespace grstapse