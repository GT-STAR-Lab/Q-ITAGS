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
#include "grstapse/geometric_planning/motion_planners/complete_euclidean_graph_motion_planner.hpp"

// Local
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"
#include "grstapse/geometric_planning/query_results/complete_euclidean_graph_motion_planner_query_result.hpp"

namespace grstapse
{
    CompleteEuclideanGraphMotionPlanner::CompleteEuclideanGraphMotionPlanner(
        const std::shared_ptr<const ParametersBase>& parameters,
        const std::shared_ptr<EuclideanGraphEnvironment>& graph)
        : SingularEuclideanGraphMotionPlannerBase(parameters, graph)
    {}

    std::shared_ptr<const MotionPlannerQueryResultBase> CompleteEuclideanGraphMotionPlanner::computeMotionPlan(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        const auto& ic = std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(initial_configuration);
        assert(ic);
        const auto& gc = std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(goal_configuration);
        assert(gc);

        if(*ic == *gc)
        {
            return std::make_shared<CompleteEuclideanGraphMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_success,
                                                                                    ic,
                                                                                    gc,
                                                                                    0);
        }

        if(auto edge = std::dynamic_pointer_cast<EuclideanGraphEnvironment>(m_environment)->findPossibleEdge(ic, gc);
           edge)
        {
            return std::make_shared<CompleteEuclideanGraphMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_success,
                                                                                    ic,
                                                                                    gc,
                                                                                    edge->cost());
        }
        // ostream operator for EuclideanGraphConfiguration and maybe generate configuration base
        throw createLogicError(fmt::format("This graph is not a complete graph. Could not find an edge from vertex "
                                           "[{0:d}, {1:f}, {2:f}] to [{3:d}, {4:f}, {5:f}]",
                                           ic->id(),
                                           ic->x(),
                                           ic->y(),
                                           gc->id(),
                                           gc->x(),
                                           gc->y()));
    }
}  // namespace grstapse