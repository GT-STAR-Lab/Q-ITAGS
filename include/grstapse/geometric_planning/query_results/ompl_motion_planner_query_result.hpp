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

// Global
#include <memory>
// Local
#include "motion_planner_query_result_base.hpp"

// External Forward Declarations
namespace ompl::geometric
{
    class PathGeometric;
}

namespace grstapse
{
    /*!
     * \brief The result from a OMPL query
     */
    class OmplMotionPlannerQueryResult : public MotionPlannerQueryResultBase
    {
       public:
        //! Constructor
        OmplMotionPlannerQueryResult(MotionPlannerQueryStatus status,
                                     const std::shared_ptr<const ompl::geometric::PathGeometric>& path);

        //! \returns The path
        [[nodiscard]] inline const std::shared_ptr<const ompl::geometric::PathGeometric>& path() const;

        //! \copydoc MotionPlanningQueryResultBase
        [[nodiscard]] float length() const final override;

       private:
        std::shared_ptr<const ompl::geometric::PathGeometric> m_path;

        friend void to_json(nlohmann::json& j, const OmplMotionPlannerQueryResult& r);
    };

    void to_json(nlohmann::json& j, const OmplMotionPlannerQueryResult& r);

    // Inline Functions
    const std::shared_ptr<const ompl::geometric::PathGeometric>& OmplMotionPlannerQueryResult::path() const
    {
        return m_path;
    }
}  // namespace grstapse