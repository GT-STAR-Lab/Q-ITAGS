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
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_base.hpp"

// External
#include <range/v3/view/iota.hpp>
// Local
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_name_scheme_base.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"

namespace grstapse
{
    DeterministicMilpSchedulerBase::DeterministicMilpSchedulerBase(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<MutexIndicators>& mutex_indicators,
        const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
        const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface)
        : MilpSchedulerBase(problem_inputs, mutex_indicators)
        , m_name_scheme(name_scheme)
        , m_task_info(problem_inputs, name_scheme, motion_planner_interface)
        , m_transition_info(m_task_info, problem_inputs, mutex_indicators, name_scheme, motion_planner_interface)
    {}

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::setupData()
    {
        // Tasks
        if(std::shared_ptr<const FailureReason> failure_reason = m_task_info.setupData(); failure_reason)
        {
            return failure_reason;
        }

        // Transitions
        if(std::shared_ptr<const FailureReason> failure_reason = m_transition_info.setupData(); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::createTaskVariables(GRBModel& model)
    {
        return m_task_info.createTaskVariables(model);
    }

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::createTaskTransitionVariables(GRBModel& model)
    {
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::createObjectiveVariables(GRBModel& model)
    {
        m_makespan =
            model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, m_name_scheme->createMakespanVariableName());
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::createTaskConstraints(GRBModel& model)
    {
        if(std::shared_ptr<const FailureReason> failure_reason = m_task_info.createTaskLowerBoundConstraints(model);
           failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::createTransitionConstraints(GRBModel& model)
    {
        if(std::shared_ptr<const FailureReason> failure_reason =
               m_transition_info.createPrecedenceTransitionConstraints(model);
           failure_reason)
        {
            return failure_reason;
        }
        if(std::shared_ptr<const FailureReason> failure_reason =
               m_transition_info.createMutexTransitionConstraints(model);
           failure_reason)
        {
            return failure_reason;
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DeterministicMilpSchedulerBase::createObjectiveConstraints(GRBModel& model)
    {
        for(unsigned int task_nr: ::ranges::views::iota(0u, m_problem_inputs->numberOfPlanTasks()))
        {
            model.addConstr(
                m_task_info.taskStartTimePointVariable(task_nr) + m_task_info.taskDuration(task_nr) - m_makespan <= 0,
                m_name_scheme->createMakespanConstraintName(task_nr));
        }
        return nullptr;
    }
}  // namespace grstapse