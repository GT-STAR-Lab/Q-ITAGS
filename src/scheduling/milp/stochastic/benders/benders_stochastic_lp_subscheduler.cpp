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
#include "grstapse/scheduling/milp/stochastic/benders/benders_stochastic_lp_subscheduler.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"

namespace grstapse
{
    BendersStochasticLpSubscheduler::BendersStochasticLpSubscheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<MutexIndicators>& mutex_indicators,
        const std::shared_ptr<std::vector<GRBVar>>& y_indicators,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme)
        : StochasticMilpSchedulerBase(problem_inputs, mutex_indicators, y_indicators, name_scheme)
    {}

    std::shared_ptr<const FailureReason> BendersStochasticLpSubscheduler::createObjectiveVariables(GRBModel& model)
    {
        m_makespan = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, constants::k_makespan);
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            if(std::shared_ptr<const FailureReason> failure_reason =
                   m_subschedulers[i]->createObjectiveVariables(model);
               failure_reason)
            {
                return failure_reason;
            }
            m_y_indicators->at(i) = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, m_name_scheme->createYIndicatorName(i));
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticLpSubscheduler::createObjectiveConstraints(GRBModel& model)
    {
        /*!
         * This variable intentionally does not follow the formatting guide
         *
         * \see https://en.wikipedia.org/wiki/Big_M_method
         */
        const double M = m_problem_inputs->scheduleWorstMakespan();
        m_y_constraints.resize(m_num_scenarios);
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            m_subschedulers[i]->createObjectiveConstraints(model);
            m_y_constraints[i] =
                model.addConstr(m_subschedulers[i]->makespanVariable() - m_makespan - M * m_y_indicators->at(i) <= 0,
                                m_name_scheme->createYConstraintName(i));
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticLpSubscheduler::createInitialCuts(
        GRBModel& model,
        GRBVar& master_makespan,
        std::vector<GRBVar>& master_y_indicators)
    {
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            model.addConstr(master_makespan >=
                            m_subschedulers[q]->longestFixedChain() * (1.0 - master_y_indicators[q]));
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> BendersStochasticLpSubscheduler::createMask(Timer& timer,
                                                                                     float timeout,
                                                                                     float gamma)
    {
        return nullptr;
    }

    unsigned int BendersStochasticLpSubscheduler::numFScenarios() const
    {
        return m_num_scenarios;
    }
}  // namespace grstapse