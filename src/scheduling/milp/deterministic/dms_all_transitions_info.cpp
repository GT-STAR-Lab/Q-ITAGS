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
#include "grstapse/scheduling/milp/deterministic/dms_all_transitions_info.hpp"

// Global
#include <ranges>
// External
#include <fmt/format.h>
// Local
#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/common/utilities/compound_failure_reason.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_tasks_info.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_name_scheme_base.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_transition_info.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"

namespace grstapse
{
    DmsAllTransitionsInfo::DmsAllTransitionsInfo(
        DmsAllTasksInfo& tasks_info,
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<MutexIndicators>& mutex_indicators,
        const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
        const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface)
        : m_tasks_info(tasks_info)
        , m_problem_inputs(problem_inputs)
        , m_mutex_indicators(mutex_indicators)
        , m_name_scheme(name_scheme)
        , m_motion_planner_interface(motion_planner_interface)
    {}

    std::shared_ptr<const FailureReason> DmsAllTransitionsInfo::setupData()
    {
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
        m_transition_infos.resize(num_tasks);
        for(auto& v: m_transition_infos)
        {
            v.resize(num_tasks, std::nullopt);
        }
        for(auto [predecessor, successor]: m_problem_inputs->precedenceConstraints())
        {
            auto coalition = m_problem_inputs->transitionCoalition(predecessor, successor);
            const std::shared_ptr<const ConfigurationBase>& initial_configuration =
                m_problem_inputs->planTask(predecessor)->terminalConfiguration();
            const std::shared_ptr<const ConfigurationBase>& terminal_configuration =
                m_problem_inputs->planTask(successor)->initialConfiguration();
            DmsTransitionInfo transition_info(std::move(coalition),
                                              predecessor,
                                              successor,
                                              initial_configuration,
                                              terminal_configuration,
                                              m_name_scheme,
                                              m_motion_planner_interface);
            if(std::shared_ptr<const FailureReason> failure_reason = transition_info.setupData(); failure_reason)
            {
                return failure_reason;
            }
            m_transition_infos[predecessor][successor] = std::move(transition_info);
        }
        for(auto [first, second, var]: m_mutex_indicators->indicators() |
                                           std::views::transform(
                                               [](const auto& iter)
                                               {
                                                   return std::tuple(iter.first.first, iter.first.second, iter.second);
                                               }))
        {
            auto coalition = m_problem_inputs->transitionCoalition(first, second);

            const std::shared_ptr<const ConfigurationBase>& forward_initial_configuration =
                m_problem_inputs->planTask(first)->terminalConfiguration();
            const std::shared_ptr<const ConfigurationBase>& forward_terminal_configuration =
                m_problem_inputs->planTask(second)->initialConfiguration();
            DmsTransitionInfo forward_transition(coalition,
                                                 first,
                                                 second,
                                                 forward_initial_configuration,
                                                 forward_terminal_configuration,
                                                 m_name_scheme,
                                                 m_motion_planner_interface);
            if(std::shared_ptr<const FailureReason> failure_reason = forward_transition.setupData(); failure_reason)
            {
                return failure_reason;
            }

            const std::shared_ptr<const ConfigurationBase>& reverse_initial_configuration =
                m_problem_inputs->planTask(second)->terminalConfiguration();
            const std::shared_ptr<const ConfigurationBase>& reverse_terminal_configuration =
                m_problem_inputs->planTask(first)->initialConfiguration();
            DmsTransitionInfo reverse_transition(coalition,
                                                 second,
                                                 first,
                                                 reverse_initial_configuration,
                                                 reverse_terminal_configuration,
                                                 m_name_scheme,
                                                 m_motion_planner_interface);
            if(std::shared_ptr<const FailureReason> failure_reason = reverse_transition.setupData(); failure_reason)
            {
                return failure_reason;
            }

            m_transition_infos[first][second] = std::move(forward_transition);
            m_transition_infos[second][first] = std::move(reverse_transition);
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DmsAllTransitionsInfo::createPrecedenceTransitionConstraints(GRBModel& model)
    {
        for(auto [predecessor_index, successor_index]: m_problem_inputs->precedenceConstraints())
        {
            GRBVar& predecessor               = m_tasks_info.taskStartTimePointVariable(predecessor_index);
            const double predecessor_duration = m_tasks_info.taskDuration(predecessor_index);
            GRBVar& successor                 = m_tasks_info.taskStartTimePointVariable(successor_index);
            m_transition_infos[predecessor_index][successor_index].value().createPrecedenceTransitionConstraint(
                model,
                predecessor,
                predecessor_duration,
                successor);
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> DmsAllTransitionsInfo::createMutexTransitionConstraints(GRBModel& model)
    {
        /*!
         * This variable intentionally does not follow the formatting guide
         *
         * \see https://en.wikipedia.org/wiki/Big_M_method
         */
        const double M = getM();
        for(auto [first, second, var]: m_mutex_indicators->indicators() |
                                           std::views::transform(
                                               [](const auto& iter)
                                               {
                                                   return std::tuple(iter.first.first, iter.first.second, iter.second);
                                               }))
        {
            // first -> second
            {
                GRBVar& predecessor              = m_tasks_info.taskStartTimePointVariable(first);
                const double first_task_duration = m_tasks_info.taskDuration(first);
                GRBVar& successor                = m_tasks_info.taskStartTimePointVariable(second);
                m_transition_infos[first][second].value().createMutexTransitionConstraint(model,
                                                                                          predecessor,
                                                                                          first_task_duration,
                                                                                          successor,
                                                                                          M * (1.0 - var));
            }

            // second -> first
            {
                GRBVar& predecessor               = m_tasks_info.taskStartTimePointVariable(second);
                const double second_task_duration = m_tasks_info.taskDuration(second);
                GRBVar& successor                 = m_tasks_info.taskStartTimePointVariable(first);
                m_transition_infos[second][first].value().createMutexTransitionConstraint(model,
                                                                                          predecessor,
                                                                                          second_task_duration,
                                                                                          successor,
                                                                                          M * var);
            }
        }

        return nullptr;
    }

    UpdateModelResult DmsAllTransitionsInfo::updateTransitionDuration(GRBModel& model,
                                                                      unsigned int first,
                                                                      unsigned int second,
                                                                      const std::shared_ptr<const Robot>& robot)
    {
        return m_transition_infos[first][second].value().updateLowerBound(robot);
    }

    double DmsAllTransitionsInfo::dualCutBetaComponent() const
    {
        /*!
         * This variable intentionally does not follow the formatting guide
         *
         * \see https://en.wikipedia.org/wiki/Big_M_method
         */
        const double M = getM();

        double rv = 0.0;
        for(auto [predecessor, successor]: m_problem_inputs->precedenceConstraints())
        {
            const double predecessor_duration = m_tasks_info.taskDuration(predecessor);
            rv += m_transition_infos[predecessor][successor]->dualCut<double>(predecessor_duration);
        }
        return rv;
    }

    double DmsAllTransitionsInfo::getM() const
    {
        return m_problem_inputs->scheduleWorstMakespan();
    }
}  // namespace grstapse