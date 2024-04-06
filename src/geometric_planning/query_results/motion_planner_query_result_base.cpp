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
#include "grstapse/geometric_planning/query_results/motion_planner_query_result_base.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"
#include "grstapse/geometric_planning/query_results/graph_motion_planner_query_result.hpp"
#include "grstapse/geometric_planning/query_results/ompl_motion_planner_query_result.hpp"

namespace grstapse
{
    MotionPlannerQueryResultBase::MotionPlannerQueryResultBase(MotionPlannerQueryStatus status,
                                                               ConfigurationType configuration_type)
        : m_status(status)
        , m_configuration_type(configuration_type)
    {}

    void to_json(nlohmann::json& j, const MotionPlannerQueryResultBase& r)
    {
        switch(r.configurationType())
        {
            case ConfigurationType::e_ompl:
            {
                j = *dynamic_cast<const OmplMotionPlannerQueryResult*>(&r);
                return;
            }
            case ConfigurationType::e_graph:
            {
                j = *dynamic_cast<const GraphMotionPlannerQueryResult*>(&r);
                return;
            }
            default:
            {
                throw createLogicError("Unknown ConfigurationType");
            }
        }
    }
}  // namespace grstapse