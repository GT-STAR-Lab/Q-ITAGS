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
#include "grstapse/geometric_planning/query_results/complete_euclidean_graph_motion_planner_query_result.hpp"

namespace grstapse
{
    CompleteEuclideanGraphMotionPlannerQueryResult::CompleteEuclideanGraphMotionPlannerQueryResult(
        MotionPlannerQueryStatus status)
        : EuclideanGraphMotionPlannerQueryResultBase(status, {}, true)
        , m_cost(std::numeric_limits<float>::quiet_NaN())
    {}

    CompleteEuclideanGraphMotionPlannerQueryResult::CompleteEuclideanGraphMotionPlannerQueryResult(
        MotionPlannerQueryStatus status,
        const std::shared_ptr<const EuclideanGraphConfiguration>& init,
        const std::shared_ptr<const EuclideanGraphConfiguration>& goal,
        float cost)
        : EuclideanGraphMotionPlannerQueryResultBase(status, {init, goal}, true)
        , m_cost(cost)
    {}

}  // namespace grstapse