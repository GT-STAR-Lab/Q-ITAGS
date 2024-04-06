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
#include "grstapse/geometric_planning/query_results/graph_motion_planner_query_result.hpp"

namespace grstapse
{
    // Forward Declarations
    class EuclideanGraphConfiguration;

    /*!
     * \brief
     */
    class EuclideanGraphMotionPlannerQueryResultBase : public GraphMotionPlannerQueryResult
    {
       public:
        //! Constructor
        EuclideanGraphMotionPlannerQueryResultBase(
            MotionPlannerQueryStatus status,
            const std::vector<std::shared_ptr<const EuclideanGraphConfiguration>>& path,
            bool is_complete);

        //! \returns The underlying path as a list of PointGraphConfiguration
        [[nodiscard]] inline const std::vector<std::shared_ptr<const EuclideanGraphConfiguration>>& path() const;

       protected:
        std::vector<std::shared_ptr<const EuclideanGraphConfiguration>> m_path;
        bool m_is_complete;

        friend void to_json(nlohmann::json& j, const EuclideanGraphMotionPlannerQueryResultBase& r);
    };

    void to_json(nlohmann::json& j, const EuclideanGraphMotionPlannerQueryResultBase& r);

    // Inline Functions
    const std::vector<std::shared_ptr<const EuclideanGraphConfiguration>>&
    EuclideanGraphMotionPlannerQueryResultBase::path() const
    {
        return m_path;
    }

}  // namespace grstapse