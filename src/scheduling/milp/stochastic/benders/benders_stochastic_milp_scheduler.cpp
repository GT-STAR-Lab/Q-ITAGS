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
#include "grstapse/scheduling/milp/stochastic/benders/benders_stochastic_milp_scheduler.hpp"

// External
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <grstapse/scheduling/milp/deterministic/subscheduler_name_scheme.hpp>
// Local
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/timer_runner.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/scheduling/schedule_base.hpp"

namespace grstapse
{
    BendersStochasticMilpScheduler::BendersStochasticMilpScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme)
        : BendersStochasticMilpSchedulerBase(problem_inputs, name_scheme)
        , m_subproblem_mutex_indicators(std::make_shared<MutexIndicators>(problem_inputs, name_scheme, false))
        , m_subproblem_y_indicators(std::make_shared<std::vector<GRBVar>>(m_num_scenarios))
        , m_subproblem(problem_inputs, m_subproblem_mutex_indicators, m_subproblem_y_indicators, name_scheme)
    {}

    void BendersStochasticMilpScheduler::makeCuts(BendersCallback& callback)
    {
        TimerRunner timer_runner("bender");
#ifdef DEBUG
        std::unordered_map<std::pair<unsigned int, unsigned int>, double> mutex_stubs;
#endif
        for(auto& [p, var]: m_mutex_indicators->indicators())
        {
            const double var_x    = callback.getSolution(var);
            GRBVar& sub_problem_v = m_subproblem_mutex_indicators->get(p);
            fixVariable(sub_problem_v, var_x > 0.5 ? 1.0 : 0.0);
#ifdef DEBUG
            mutex_stubs[p] = var_x > 0.5 ? 1.0 : 0.0;
#endif
        }

#ifdef DEBUG
        std::vector<double> y_stubs(m_num_scenarios);
#endif
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            const double var_x    = callback.getSolution(m_master_y_indicators[i]);
            GRBVar& sub_problem_v = m_subproblem_y_indicators->at(i);
            fixVariable(sub_problem_v, var_x > 0.5 ? 1.0 : 0.0);
#ifdef DEBUG
            y_stubs[i] = var_x > 0.5 ? 1.0 : 0.0;
#endif
        }
        std::shared_ptr<MilpSolverResult> result = m_subproblem.resolve(true);
        if(result->failure())
        {
            throw createLogicError("Optimization for the sub-problem failed");
        }

        // Adjust upperbound
        const double primal_objective = variableValue(result->model(), constants::k_makespan);
        callback.addLazy(m_alpha_robust_makespan <= primal_objective);

#ifdef DEBUG
        const double dual_objective = m_subproblem.dualCut<double, double>(mutex_stubs, y_stubs);
        //  The dual value and the alpha_robust_makespan should be the same
        if(std::abs(primal_objective - dual_objective) > 1e-4)
        {
            throw createLogicError("Dual objective is not the same as primal objective");
        }
#endif

        // Adjust lower bound
        GRBLinExpr dual_optimality_cut =
            m_subproblem.dualCut<GRBLinExpr, GRBVar>(m_mutex_indicators->indicators(), m_master_y_indicators);
        callback.addLazy(m_alpha_robust_makespan >= dual_optimality_cut);
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpScheduler::setupData()
    {
        std::shared_ptr<MilpSolverResult> result = m_subproblem.createModel(m_problem_inputs->schedulerParameters());
        if(!result->success())
        {
            return result->failureReason();
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticMilpScheduler::createInitialCuts(GRBModel& model)
    {
        return m_subproblem.createInitialCuts(model, m_alpha_robust_makespan, m_master_y_indicators);
    }

}  // namespace grstapse