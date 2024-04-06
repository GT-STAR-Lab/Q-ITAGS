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
#include <stdexcept>
#include <vector>
// Local
#include "grstapse/scheduling/scheduler_motion_planner_interface_base.hpp"

namespace grstapse
{
    /*!
     * The standard interface between scheduling and motion planning
     */
    class CommonSchedulerMotionPlannerInterface : public SchedulerMotionPlannerInterfaceBase
    {
       public:
        //! \copydoc SchedulerMotionPlannerInterfaceBase
        float computeTaskDuration(const std::shared_ptr<const Task>& task,
                                  const std::vector<std::shared_ptr<const Robot>>& coalition) const final override;

        //! \copydoc SchedulerMotionPlannerInterfaceBase
        bool isInitialTransitionMemoized(const std::shared_ptr<const ConfigurationBase>& configuration,
                                         const std::shared_ptr<const Robot>& robot) const final override;

        //! \copydoc SchedulerMotionPlannerInterfaceBase
        float computeInitialTransitionDuration(const std::shared_ptr<const ConfigurationBase>& configuration,
                                               const std::shared_ptr<const Robot>& robot) const final override;

        //! \copydoc SchedulerMotionPlannerInterfaceBase
        bool isTransitionMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                                  const std::shared_ptr<const ConfigurationBase>& goal,
                                  const std::shared_ptr<const Robot>& robot) const final override;

        //! \copydoc SchedulerMotionPlannerInterfaceBase
        float computeTransitionDuration(const std::shared_ptr<const ConfigurationBase>& initial,
                                        const std::shared_ptr<const ConfigurationBase>& goal,
                                        const std::shared_ptr<const Robot>& robot) const final override;
    };

}  // namespace grstapse