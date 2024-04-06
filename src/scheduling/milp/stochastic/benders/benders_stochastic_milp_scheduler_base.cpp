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
#include "grstapse/scheduling/milp/stochastic/benders/benders_stochastic_milp_scheduler_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_schedule.hpp"

namespace grstapse
{
    BendersStochasticMilpSchedulerBase::BendersStochasticMilpSchedulerBase(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme)
        : MilpSchedulerBase(problem_inputs, std::make_shared<MutexIndicators>(problem_inputs, name_scheme), true)
        , m_num_scenarios(problem_inputs->schedulerParameters()->get<unsigned int>(constants::k_num_scenarios))
        , m_alpha_q(m_num_scenarios * problem_inputs->schedulerParameters()->get<float>(constants::k_gamma))
        , m_task_stubs(m_num_scenarios)
        , m_master_y_indicators(m_num_scenarios)
        , m_name_scheme(name_scheme)
    {}

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createTaskVariables(GRBModel& model)
    {
        for(unsigned int task_nr = 0, num_tasks = m_problem_inputs->numberOfPlanTasks(); task_nr < num_tasks; ++task_nr)
        {
            m_task_stubs[task_nr] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createTaskTransitionVariables(
        GRBModel& model)
    {
        // Intentionally does nothing
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createObjectiveVariables(GRBModel& model)
    {
        m_alpha_robust_makespan = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, constants::k_makespan);
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            // Note: name overlap with the subproblem y indicators doesn't matter because they are two different models
            m_master_y_indicators[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, m_name_scheme->createYIndicatorName(i));
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createObjective(GRBModel& model)
    {
        // Set all optimization to minimize (is the default, but we explicitly set anyway)
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        model.setObjective(GRBLinExpr(m_alpha_robust_makespan));
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createTaskConstraints(GRBModel& model)
    {
        // Intentionally does nothing
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createTransitionConstraints(
        GRBModel& model)
    {
        // pseudo-ordering constraints
        for(auto [predecessor, successor]: m_problem_inputs->precedenceConstraints())
        {
            // There are no strict inequality constraints so the 1's are to make pseudo-strict inequality
            model.addConstr(m_task_stubs[predecessor] + 1 <= m_task_stubs[successor]);
        }
        const double M = m_problem_inputs->scheduleWorstMakespan();
        for(auto& [p, v]: m_mutex_indicators->indicators())
        {
            // There are no strict inequality constraints so the 1's are to make pseudo-strict inequality
            model.addConstr(m_task_stubs[p.first] + 1 - m_task_stubs[p.second] - M * (1.0 - v) <= 0);
            model.addConstr(m_task_stubs[p.second] + 1 - m_task_stubs[p.first] - M * v <= 0);
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpSchedulerBase::createObjectiveConstraints(GRBModel& model)
    {
        GRBLinExpr y_sum;
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            y_sum += m_master_y_indicators[i];
        }
        model.addConstr(m_alpha_q >= y_sum, "y_sum");

        if(std::shared_ptr<const FailureReason> failure_reason = createInitialCuts(model); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const ScheduleBase> BendersStochasticMilpSchedulerBase::createSchedule(GRBModel& model)
    {
        const double makespan = m_alpha_robust_makespan.get(GRB_DoubleAttr_X);
        std::vector<std::pair<unsigned int, unsigned int>> precedence_set_mutex_constraints =
            m_mutex_indicators->precedenceSet();
        return std::make_shared<const StochasticSchedule>(makespan, precedence_set_mutex_constraints);
    }

}  // namespace grstapse