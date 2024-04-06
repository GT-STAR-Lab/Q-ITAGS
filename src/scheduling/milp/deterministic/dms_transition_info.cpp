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
#include "grstapse/scheduling/milp/deterministic/dms_transition_info.hpp"

// Local
#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_name_scheme_base.hpp"
#include "grstapse/scheduling/scheduler_motion_planner_interface_base.hpp"
#include "grstapse/scheduling/transition_failure.hpp"
#include "grstapse/species.hpp"

namespace grstapse
{
    DmsTransitionInfo::DmsTransitionInfo(
        CoalitionView coalition,
        unsigned int predecessor_index,
        unsigned int successor_index,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& terminal_configuration,
        const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
        const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface)
        : m_predecessor_index(predecessor_index)
        , m_successor_index(successor_index)
        , m_initial_configuration(initial_configuration)
        , m_terminal_configuration(terminal_configuration)
        , m_name_scheme(name_scheme)
        , m_motion_planner_interface(motion_planner_interface)
        , m_duration_lowerbound(0.0f)
    {
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            m_coalition[robot] =
                std::pair(TransitionComputationStatus::e_none, std::numeric_limits<float>::quiet_NaN());
        }
    }

    std::shared_ptr<const FailureReason> DmsTransitionInfo::setupData()
    {
        for(auto& [robot, transition_status]: m_coalition)
        {
            // Compute Transition Data
            float transition_duration;
            if(m_motion_planner_interface->isTransitionMemoized(m_initial_configuration,
                                                                m_terminal_configuration,
                                                                robot))
            {
                transition_duration = m_motion_planner_interface->computeTransitionDuration(m_initial_configuration,
                                                                                            m_terminal_configuration,
                                                                                            robot);
                if(transition_duration < 0.0f)
                {
                    return std::shared_ptr<const TransitionFailure>(
                        new TransitionFailure{{.species                = robot->species()->name(),
                                               .predecessor_task_index = m_predecessor_index,
                                               .successor_task_index   = m_successor_index}});
                }
                transition_status.first = TransitionComputationStatus::e_success;
            }
            else
            {
                transition_duration =
                    m_motion_planner_interface->computeTransitionDurationHeuristic(m_initial_configuration,
                                                                                   m_terminal_configuration,
                                                                                   robot);

                transition_status.first = TransitionComputationStatus::e_heuristic;
            }
            transition_status.second = transition_duration;
            m_duration_lowerbound    = std::max(m_duration_lowerbound, transition_duration);
        }
        return nullptr;
    }

    void DmsTransitionInfo::createPrecedenceTransitionConstraint(GRBModel& model,
                                                                 GRBVar& predecessor,
                                                                 double predecessor_duration,
                                                                 GRBVar& successor)
    {
        m_transition_name = m_name_scheme->createPrecedenceConstraintName(m_predecessor_index, m_successor_index);
        m_transition_constraint =
            model.addConstr(predecessor - successor + predecessor_duration <= -m_duration_lowerbound,
                            m_transition_name);
    }

    void DmsTransitionInfo::createMutexTransitionConstraint(GRBModel& model,
                                                            GRBVar& predecessor,
                                                            double predecessor_duration,
                                                            GRBVar& successor,
                                                            GRBLinExpr&& mutex_indicator_component)
    {
        m_transition_name       = m_name_scheme->createMutexConstraintName(m_predecessor_index, m_successor_index);
        m_transition_constraint = model.addConstr(
            predecessor - successor + predecessor_duration - mutex_indicator_component <= -m_duration_lowerbound,
            m_transition_name);
    }

    UpdateModelResult DmsTransitionInfo::updateLowerBound(const std::shared_ptr<const Robot>& robot)
    {
        std::pair<TransitionComputationStatus, float>& transition_status = m_coalition[robot];
        if(transition_status.first == TransitionComputationStatus::e_success)
        {
            return UpdateModelResult(UpdateModelResultType::e_no_update);
        }

        const float computed_transition_duration =
            m_motion_planner_interface->computeTransitionDuration(m_initial_configuration,
                                                                  m_terminal_configuration,
                                                                  robot);
        if(computed_transition_duration < 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const TransitionFailure>(
                new TransitionFailure{{.species                = robot->species()->name(),
                                       .predecessor_task_index = m_predecessor_index,
                                       .successor_task_index   = m_successor_index}}));
        }

        transition_status.first  = TransitionComputationStatus::e_success;
        transition_status.second = computed_transition_duration;
        if(computed_transition_duration > m_duration_lowerbound)
        {
            m_duration_lowerbound = computed_transition_duration;
            m_transition_constraint.set(GRB_DoubleAttr_RHS, -m_duration_lowerbound);
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }

        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }
}  // namespace grstapse