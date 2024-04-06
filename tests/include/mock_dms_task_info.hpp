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

// Project
#include <grstapse/scheduling/milp/deterministic/dms_task_info.hpp>

namespace grstapse::mocks
{
    class MockDmsTaskInfo : public DmsTaskInfo
    {
       public:
        MockDmsTaskInfo(CoalitionView coalition,
                        unsigned int plan_task_nr,
                        const std::shared_ptr<const Task>& task,
                        const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
                        const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface)
            : DmsTaskInfo(coalition, plan_task_nr, task, name_scheme, motion_planner_interface)
        {}

        void setLowerbound(float lowerbound)
        {
            m_lower_bound = lowerbound;
        }
    };

}  // namespace grstapse::mocks