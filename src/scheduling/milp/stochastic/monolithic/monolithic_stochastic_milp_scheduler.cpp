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
#include "grstapse/scheduling/milp/stochastic/monolithic/monolithic_stochastic_milp_scheduler.hpp"

// Global
#include <coroutine>
// Local
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/timeout_failure.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/sequential_probability_ratio_test.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_schedule.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"

namespace grstapse
{
    MonolithicStochasticMilpScheduler::MonolithicStochasticMilpScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : StochasticMilpSchedulerBase(problem_inputs)
    {}

    std::shared_ptr<const FailureReason> MonolithicStochasticMilpScheduler::createObjectiveConstraints(GRBModel& model)
    {
        GRBLinExpr y_summation;
        const double M = m_problem_inputs->scheduleWorstMakespan();
        for(unsigned int q = 0; q < m_num_scenarios; ++q)
        {
            m_subschedulers[q]->createObjectiveConstraints(model);

            y_summation += m_y_indicators->at(q);

            model.addConstr(m_subschedulers[q]->makespanVariable() - m_makespan - M * m_y_indicators->at(q) <= 0,
                            m_name_scheme->createYConstraintName(q));
        }
        model.addConstr(m_alpha_q >= y_summation, "y_summation");
        return nullptr;
    }

    std::shared_ptr<const FailureReason> MonolithicStochasticMilpScheduler::createMask(Timer& timer,
                                                                                       float timeout,
                                                                                       float gamma)
    {
        return nullptr;
    }

    unsigned int MonolithicStochasticMilpScheduler::numFScenarios() const
    {
        return m_num_scenarios;
    }
}  // namespace grstapse