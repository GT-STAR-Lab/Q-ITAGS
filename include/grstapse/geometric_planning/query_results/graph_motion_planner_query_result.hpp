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
#pragma once

// Local
#include "grstapse/geometric_planning/query_results/motion_planner_query_result_base.hpp"

namespace grstapse
{
    // Forward Declaration
    enum class GraphType : uint8_t;

    //! Base class for the motion planner query results for a graph search
    class GraphMotionPlannerQueryResult : public MotionPlannerQueryResultBase
    {
       public:
        //! \returns The graph type
        [[nodiscard]] inline GraphType graphType() const;

       protected:
        //! Constructor
        explicit GraphMotionPlannerQueryResult(MotionPlannerQueryStatus status, GraphType graph_type);

        GraphType m_graph_type;
    };

    void to_json(nlohmann::json& j, const GraphMotionPlannerQueryResult& r);

    GraphType GraphMotionPlannerQueryResult::graphType() const
    {
        return m_graph_type;
    }

}  // namespace grstapse