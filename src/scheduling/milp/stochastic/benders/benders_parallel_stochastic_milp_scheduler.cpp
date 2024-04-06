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
#include "grstapse/scheduling/milp/stochastic/benders/benders_parallel_stochastic_milp_scheduler.hpp"

// External
#include <omp.h>
// Local
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"

namespace grstapse
{
    BendersParallelStochasticMilpScheduler::BendersParallelStochasticMilpScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme)
        : BendersStochasticMilpSchedulerBase(problem_inputs, name_scheme)
        , m_y_indicator_values(m_num_scenarios)
        , m_subproblem_makespans(m_num_scenarios)
    {}

    void BendersParallelStochasticMilpScheduler::makeCuts(MilpSolverBase::BendersCallback& callback)
    {
        for(auto& [p, var]: m_mutex_indicators->indicators())
        {
            const double var_x          = callback.getSolution(var);
            m_mutex_indicator_values[p] = var_x > 0.5;
        }
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            const double var_x      = callback.getSolution(m_master_y_indicators[q]);
            m_y_indicator_values[q] = var_x > 0.5;
        }

#pragma omp parallel for shared(m_subschedulers,                                                                       \
                                m_mutex_indicator_values,                                                              \
                                m_y_indicator_values,                                                                  \
                                m_subproblem_mutex_indicators,                                                         \
                                m_subproblem_makespans)
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            //            if(m_y_indicator_values[q])
            //            {
            //                m_subproblem_makespans[q] = std::numeric_limits<float>::infinity();
            //                continue;
            //            }

            for(auto& [p, var]: m_subproblem_mutex_indicators[q]->indicators())
            {
                fixVariable(var, m_mutex_indicator_values.at(p) ? 1.0 : 0.0);
            }

            std::shared_ptr<MilpSolverResult> result = m_subschedulers[q]->resolve(true);
            if(result->failure())
            {
                m_subproblem_makespans[q] = -1.0f;
                continue;
            }

            m_subproblem_makespans[q] = variableValue(m_subschedulers[q]->makespanVariable());
        }

        if(std::any_of(m_subproblem_makespans.begin(),
                       m_subproblem_makespans.end(),
                       [](float v) -> bool
                       {
                           return v < 0.0f;
                       }))
        {
            throw createLogicError("sub-problem failed");
        }

#ifdef DEBUG
        std::unordered_map<std::pair<unsigned int, unsigned int>, double> mutex_indicators_stubs;
        for(auto [p, v]: m_mutex_indicator_values)
        {
            mutex_indicators_stubs.emplace(p, v ? 1.0 : 0.0);
        }
#endif

        const double M = getM();
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            const double primal_objective = m_subproblem_makespans[q];
//            if(!m_y_indicator_values[q])
//            {
//                callback.addLazy(m_alpha_robust_makespan <= primal_objective);
//            }
#ifdef DEBUG

            const double dual_objective = m_subschedulers[q]->dualCut<double, double>(mutex_indicators_stubs);
            if(std::abs(primal_objective - dual_objective) > 1e-3)
            {
                throw createLogicError(fmt::format("Primal ({}) and Dual ({}) do not have the same objective",
                                                   primal_objective,
                                                   dual_objective));
            }
#endif

            GRBLinExpr dual_cut = m_subschedulers[q]->dualCut<GRBLinExpr, GRBVar>(m_mutex_indicators->indicators());
            dual_cut -= M * m_master_y_indicators[q];
            callback.addLazy(m_alpha_robust_makespan >= dual_cut);
        }
    }

    std::shared_ptr<const FailureReason> BendersParallelStochasticMilpScheduler::setupData()
    {
        m_subproblem_mutex_indicators.reserve(m_num_scenarios);
        m_subschedulers.reserve(m_num_scenarios);
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            m_subproblem_mutex_indicators.push_back(
                std::make_shared<MutexIndicators>(m_problem_inputs, m_name_scheme, false));
            m_subschedulers.emplace_back(
                std::make_unique<DeterministicMilpSubscheduler>(q,
                                                                m_problem_inputs,
                                                                m_subproblem_mutex_indicators.back(),
                                                                false));
            if(std::shared_ptr<MilpSolverResult> result =
                   m_subschedulers.back()->createModel(m_problem_inputs->schedulerParameters());
               result->failure())
            {
                return result->failureReason();
            }
        }

        return nullptr;
    }
    std::shared_ptr<const FailureReason> BendersParallelStochasticMilpScheduler::createInitialCuts(GRBModel& model)
    {
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            model.addConstr(m_alpha_robust_makespan >=
                            m_subschedulers[q]->longestFixedChain() * (1.0 - m_master_y_indicators[q]));
        }
        return nullptr;
    }
}  // namespace grstapse