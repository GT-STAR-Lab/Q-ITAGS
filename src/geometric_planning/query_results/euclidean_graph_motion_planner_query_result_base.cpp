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
#include "grstapse/geometric_planning/query_results/euclidean_graph_motion_planner_query_result_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    EuclideanGraphMotionPlannerQueryResultBase::EuclideanGraphMotionPlannerQueryResultBase(
        MotionPlannerQueryStatus status,
        const std::vector<std::shared_ptr<const EuclideanGraphConfiguration>>& path,
        bool is_complete)
        : GraphMotionPlannerQueryResult(status, GraphType::e_euclidean)
        , m_path(path)
        , m_is_complete(is_complete)
    {}

    void to_json(nlohmann::json& j, const EuclideanGraphMotionPlannerQueryResultBase& r)
    {
        j = {{constants::k_path, r.path()}, {constants::k_path_length, (&r)->length()}};
    }
}  // namespace grstapse