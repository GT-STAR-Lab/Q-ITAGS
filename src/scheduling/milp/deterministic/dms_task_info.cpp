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
#include "grstapse/scheduling/milp/deterministic/dms_task_info.hpp"

// Global
#include <string_view>
// External
#include <range/v3/all.hpp>
// Local
#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/common/utilities/compound_failure_reason.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/std_extension.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/initial_transition_failure.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_name_scheme_base.hpp"
#include "grstapse/scheduling/scheduler_motion_planner_interface_base.hpp"
#include "grstapse/scheduling/task_duration_failure.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    DmsTaskInfo::DmsTaskInfo(CoalitionView coalition,
                             unsigned int plan_task_nr,
                             const std::shared_ptr<const Task>& task,
                             const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
                             const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface)
        : m_plan_task_nr(plan_task_nr)
        , m_task(task)
        , m_name_scheme(name_scheme)
        , m_motion_planner_interface(motion_planner_interface)
        , m_duration(0.0f)
        , m_lower_bound(0.0f)
    {
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            m_coalition[robot] =
                std::pair(TransitionComputationStatus::e_none, std::numeric_limits<float>::quiet_NaN());
        }
    }

    std::shared_ptr<const FailureReason> DmsTaskInfo::setupData()
    {
        if(m_coalition.empty())
        {
            m_lower_bound = 0.0f;
            m_duration    = m_task->staticDuration();
            return nullptr;
        }

        const std::shared_ptr<const ConfigurationBase>& initial_configuration  = m_task->initialConfiguration();
        const std::shared_ptr<const ConfigurationBase>& terminal_configuration = m_task->terminalConfiguration();
        std::vector<std::shared_ptr<const Robot>> coalition;
        coalition.reserve(m_coalition.size());
        for(auto& [robot, transition_status]: m_coalition)
        {
            coalition.push_back(robot);

            // Compute Initial Transition Data
            float initial_transition_duration;
            if(m_motion_planner_interface->isInitialTransitionMemoized(initial_configuration, robot))
            {
                initial_transition_duration =
                    m_motion_planner_interface->computeInitialTransitionDuration(initial_configuration, robot);
                if(initial_transition_duration < 0.0f)
                {
                    return std::shared_ptr<const InitialTransitionFailure>(
                        new InitialTransitionFailure{{.robot = robot->id(), .task = m_plan_task_nr}});
                }
                transition_status.first = TransitionComputationStatus::e_success;
            }
            else
            {
                initial_transition_duration =
                    m_motion_planner_interface->computeInitialTransitionDurationHeuristic(initial_configuration, robot);
                transition_status.first = TransitionComputationStatus::e_heuristic;
            }
            transition_status.second = initial_transition_duration;
            m_lower_bound            = std::max(m_lower_bound, initial_transition_duration);
        }

        m_duration = m_motion_planner_interface->computeTaskDuration(m_task, coalition);
        if(m_duration < 0.0f)
        {
            std::vector<std::shared_ptr<const FailureReason>> reasons;
            reasons.reserve(m_coalition.size());
            for(const std::shared_ptr<const Robot>& robot: coalition)
            {
                reasons.push_back(std::shared_ptr<TaskDurationFailure>(
                    new TaskDurationFailure{{.species = robot->species()->name(), .task = m_plan_task_nr}}));
            }
            return std::make_shared<CompoundFailureReason>(reasons);
        }

        return nullptr;
    }

    void DmsTaskInfo::createTimePointVariables(GRBModel& model)
    {
        m_start_time_point = model.addVar(-GRB_INFINITY,
                                          GRB_INFINITY,
                                          0.0,
                                          GRB_CONTINUOUS,
                                          m_name_scheme->createTaskStartName(m_plan_task_nr));
    }

    void DmsTaskInfo::createLowerBoundConstraint(GRBModel& model)
    {
        m_lower_bound_constraint =
            model.addConstr(-m_start_time_point <= -m_lower_bound,
                            m_name_scheme->createTaskStartLowerBoundConstraintName(m_plan_task_nr));
    }

    UpdateModelResult DmsTaskInfo::updateLowerBound(const std::shared_ptr<const Robot>& robot)
    {
        std::pair<TransitionComputationStatus, float>& transition_status = m_coalition[robot];
        if(transition_status.first == TransitionComputationStatus::e_success)
        {
            return UpdateModelResult(UpdateModelResultType::e_no_update);
        }

        const std::shared_ptr<const ConfigurationBase>& initial_configuration = m_task->initialConfiguration();
        const float initial_transition_duration =
            m_motion_planner_interface->computeInitialTransitionDuration(initial_configuration, robot);
        if(initial_transition_duration < 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const InitialTransitionFailure>(
                new InitialTransitionFailure{{.robot = robot->id(), .task = m_plan_task_nr}}));
        }

        transition_status.first  = TransitionComputationStatus::e_success;
        transition_status.second = initial_transition_duration;
        if(initial_transition_duration > m_lower_bound)
        {
            m_lower_bound = initial_transition_duration;
            m_lower_bound_constraint.set(GRB_DoubleAttr_RHS, -m_lower_bound);
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }

        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    std::vector<std::shared_ptr<const Robot>> DmsTaskInfo::coalition() const
    {
        return m_coalition | ranges::views::keys | ranges::to<std::vector<std::shared_ptr<const Robot>>>();
    }

    double DmsTaskInfo::dualCut() const
    {
        const double eta = constraintDualValue(m_lower_bound_constraint);
        return m_lower_bound * eta;
    }
}  // namespace grstapse