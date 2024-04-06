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
#include "grstapse/scheduling/milp/deterministic/dms_all_tasks_info.hpp"

// Local
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_name_scheme_base.hpp"
#include "grstapse/scheduling/scheduler_motion_planner_interface_base.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    DmsAllTasksInfo::DmsAllTasksInfo(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
        const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& scheduler_motion_planner_interface)
        : m_problem_inputs(problem_inputs)
        , m_name_scheme(name_scheme)
        , m_scheduler_motion_planner_interface(scheduler_motion_planner_interface)
    {}

    std::shared_ptr<const FailureReason> DmsAllTasksInfo::setupData()
    {
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
        m_task_infos.reserve(num_tasks);
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            auto coalition = m_problem_inputs->coalition(task_nr);
            m_task_infos.emplace_back(coalition,
                                      task_nr,
                                      m_problem_inputs->planTask(task_nr),
                                      m_name_scheme,
                                      m_scheduler_motion_planner_interface);
            if(std::shared_ptr<const FailureReason> failure_reason = m_task_infos.back().setupData(); failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DmsAllTasksInfo::createTaskVariables(GRBModel& model)
    {
        for(DmsTaskInfo& task_info: m_task_infos)
        {
            task_info.createTimePointVariables(model);
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DmsAllTasksInfo::createTaskLowerBoundConstraints(GRBModel& model)
    {
        for(DmsTaskInfo& task_info: m_task_infos)
        {
            task_info.createLowerBoundConstraint(model);
        }
        return nullptr;
    }

    UpdateModelResult DmsAllTasksInfo::updateTaskLowerBound(unsigned int task_nr,
                                                            const std::shared_ptr<const Robot>& robot)
    {
        return m_task_infos[task_nr].updateLowerBound(robot);
    }

    std::vector<unsigned int> DmsAllTasksInfo::scheduledOrder() const
    {
        std::vector<unsigned int> rv(m_task_infos.size());
        for(unsigned int i = 0, end = m_task_infos.size(); i < end; ++i)
        {
            rv[i] = i;
        }
        std::sort(rv.begin(),
                  rv.end(),
                  [this](unsigned int lhs, unsigned int rhs)
                  {
                      return m_task_infos[lhs].startTimePoint().get(GRB_DoubleAttr_X) <
                             m_task_infos[rhs].startTimePoint().get(GRB_DoubleAttr_X);
                  });
        return rv;
    }

    std::vector<std::pair<float, float>> DmsAllTasksInfo::timePoints() const
    {
        std::vector<std::pair<float, float>> rv;
        const unsigned int num_tasks = m_task_infos.size();
        rv.reserve(num_tasks);
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            const float start    = m_task_infos[task_nr].startTimePoint().get(GRB_DoubleAttr_X);
            const float duration = m_task_infos[task_nr].duration();
            rv.push_back(std::pair(start, start + duration));
        }
        return rv;
    }

    double DmsAllTasksInfo::dualCut() const
    {
        double rv = 0.0;
        for(const DmsTaskInfo& task: m_task_infos)
        {
            rv += task.dualCut();
        }
        return rv;
    }
}  // namespace grstapse