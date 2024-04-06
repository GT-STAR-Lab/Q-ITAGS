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
#include "grstapse/geometric_planning/motion_planners/complete_sampled_euclidean_graph_motion_planner.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/complete_euclidean_graph_motion_planner.hpp"

namespace grstapse
{
    CompleteSampledEuclideanGraphMotionPlanner::CompleteSampledEuclideanGraphMotionPlanner(
        const std::shared_ptr<const ParametersBase>& parameters,
        const std::shared_ptr<SampledEuclideanGraphEnvironment>& environment)
        : SampledEuclideanGraphMotionPlannerBase(parameters, environment)
    {
        m_sub_motion_planners.reserve(environment->numGraphs());
        for(unsigned int i = 0, end = environment->numGraphs(); i < end; ++i)
        {
            m_sub_motion_planners.push_back(std::dynamic_pointer_cast<SingularEuclideanGraphMotionPlannerBase>(
                std::make_shared<CompleteEuclideanGraphMotionPlanner>(parameters, environment->graph(i))));
        }
    }
}  // namespace grstapse