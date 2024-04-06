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
#include "grstapse/geometric_planning/motion_planners/singular_euclidean_graph_motion_planner_base.hpp"

namespace grstapse
{
    //! A motion planner for a complete euclidean graph
    class CompleteEuclideanGraphMotionPlanner : public SingularEuclideanGraphMotionPlannerBase
    {
       public:
        //! Constructor
        CompleteEuclideanGraphMotionPlanner(const std::shared_ptr<const ParametersBase>& parameters,
                                            const std::shared_ptr<EuclideanGraphEnvironment>& graph);

        //! \copydoc MotionPlannerBase
        [[nodiscard]] inline bool isMemoized(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) const final override;

       protected:
        //! \copydoc MotionPlannerBase
        std::shared_ptr<const MotionPlannerQueryResultBase> computeMotionPlan(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) final override;
    };

    // Inline Functions
    bool CompleteEuclideanGraphMotionPlanner::isMemoized(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration) const
    {
        return true;
    }

}  // namespace grstapse