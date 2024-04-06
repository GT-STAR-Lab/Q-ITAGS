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
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/heuristic_approximation_stochastic_scheduler.hpp"

// region Includes

// External
#include <range/v3/view/enumerate.hpp>
// Local
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/common/utilities/timeout_failure.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/masked_complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/scenario_selector_base.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_schedule.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"
// endregion

namespace grstapse
{
    HeuristicApproximationStochasticScheduler::HeuristicApproximationStochasticScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : StochasticMilpSchedulerBase(problem_inputs)
        , m_scenario_selector(std::make_shared<HeuristicScenarioSelector>(problem_inputs))
    {
        m_num_scenarios = m_problem_inputs->schedulerParameters()->get<unsigned int>(constants::k_beta);
        m_num_f_samples = m_problem_inputs->schedulerParameters()->get<float>(constants::k_num_scenarios);
    }

    HeuristicApproximationStochasticScheduler::HeuristicApproximationStochasticScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::function<std::shared_ptr<ScenarioSelectorBase>(
            const std::shared_ptr<const SchedulerProblemInputs>&)>& scenario_selector_constructor)
        : StochasticMilpSchedulerBase(problem_inputs)
        , m_scenario_selector(scenario_selector_constructor(problem_inputs))
    {
        m_num_scenarios = m_problem_inputs->schedulerParameters()->get<unsigned int>(constants::k_beta);
        m_num_f_samples = m_problem_inputs->schedulerParameters()->get<float>(constants::k_num_scenarios);
    }

    std::shared_ptr<const FailureReason> HeuristicApproximationStochasticScheduler::createMask(Timer& timer,
                                                                                               float timeout,
                                                                                               float gamma)
    {
        std::optional<std::vector<bool>> mask =
            m_scenario_selector->createMask(timer, m_motion_planner, m_num_f_samples, m_num_scenarios, gamma, timeout);
        if(not mask)
        {
            Logger::warn("Scheduler timed out");
            return std::make_shared<TimeoutFailure>();
        }
        m_motion_planner->setMask(mask.value());
        return nullptr;
    }

    std::shared_ptr<const FailureReason> HeuristicApproximationStochasticScheduler::createObjectiveVariables(
        GRBModel& model)
    {
        // Same as parent version of the function, but doesn't create y indicators
        m_makespan =
            model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, m_name_scheme->createMakespanVariableName());
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            if(std::shared_ptr<const FailureReason> failure_reason =
                   m_subschedulers[i]->createObjectiveVariables(model);
               failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> HeuristicApproximationStochasticScheduler::createObjectiveConstraints(
        GRBModel& model)
    {
        // No constraints on y indicators
        for(auto&& [q, subscheduler]: m_subschedulers | ::ranges::views::enumerate)
        {
            subscheduler->createObjectiveConstraints(model);
            model.addConstr(subscheduler->makespanVariable() - m_makespan <= 0,
                            m_name_scheme->createYConstraintName(q));
        }

        return nullptr;
    }

    unsigned int HeuristicApproximationStochasticScheduler::numFScenarios() const
    {
        return m_num_f_samples;
    }
}  // namespace grstapse