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
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/heuristic_scenario_selector.hpp"

// Global
#include <memory>
#include <random>
#include <set>
// External
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/take.hpp>
// Local
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/common/utilities/timeout_failure.hpp"
#include "grstapse/common/utilities/timer.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/masked_complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"

namespace grstapse
{
    HeuristicScenarioSelector::HeuristicScenarioSelector(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : ScenarioSelectorBase(problem_inputs)
    {}

    std::optional<std::vector<bool>> HeuristicScenarioSelector::createMask(
        Timer& timer,
        const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
        unsigned int num_samples,
        unsigned int beta,
        float gamma,
        float timeout)
    {
        std::vector<std::pair<unsigned int, unsigned int>> task_edges;
        std::vector<std::pair<unsigned int, unsigned int>> precedence_transition_edges;
        std::vector<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>> mutex_transition_edges;

        task_edges.reserve(m_problem_inputs->numberOfPlanTasks());
        for(auto task: m_problem_inputs->planTasks())
        {
            task_edges.emplace_back(ScenarioSelectorBase::getEdge(task));
        }

        //        const auto& precedence_constraints = m_problem_inputs->precedenceConstraints();
        //
        //        precedence_transition_edges.reserve(precedence_constraints.size());
        //        mutex_transition_edges.reserve(m_problem_inputs->mutexConstraints().size());
        //        for(const auto [i, j]: m_problem_inputs->mutexConstraints())
        //        {
        //            if(precedence_constraints.contains({i, j}))
        //            {
        //                precedence_transition_edges.emplace_back(getEdge(i, j));
        //            }
        //            else if(precedence_constraints.contains({j, i}))
        //            {
        //                precedence_transition_edges.emplace_back(getEdge(j, i));
        //            }
        //            else
        //            {
        //                auto i_to_j = getEdge(i, j);
        //                auto j_to_i = getEdge(j, i);
        //                mutex_transition_edges.emplace_back(
        //                    std::tuple{i_to_j.first, i_to_j.second, j_to_i.first, j_to_i.second});
        //            }
        //        }
        if(timer.get() > timeout)
        {
            // Log one level up
            return std::nullopt;
        }

        std::set<std::pair<float, unsigned int>> label_map =
            std::dynamic_pointer_cast<SampledEuclideanGraphEnvironment>(motion_planner->environment())->graphs() |
            ::ranges::views::take(num_samples) |
            ::ranges::views::transform(
                [this, &task_edges, &precedence_transition_edges, &mutex_transition_edges](
                    const std::shared_ptr<EuclideanGraphEnvironment>& environment) -> float
                {
                    return this->label(environment, task_edges, precedence_transition_edges, mutex_transition_edges);
                }) |
            ::ranges::views::enumerate |
            ::ranges::views::transform(
                [](auto&& p) -> std::pair<float, unsigned int>
                {
                    return {p.second, p.first};
                }) |
            ::ranges::to<std::set>();
        if(timer.get() > timeout)
        {
            return std::nullopt;
        }

        const unsigned int num_h = static_cast<unsigned int>(num_samples * (1.0 - gamma) + 0.5);
        std::set<unsigned int> sampled;
        {
            std::random_device random_device;
            std::default_random_engine random_engine(random_device());
            std::uniform_int_distribution<unsigned int> uniform_int_distribution(0, num_h - 1);
            sampled.insert(num_h - 1);
            while(sampled.size() < beta)
            {
                sampled.insert(uniform_int_distribution(random_engine));
            }
        }

        std::vector<bool> mask(num_samples, false);
        for(unsigned int i: label_map | ::ranges::views::take(num_h) | ::ranges::views::enumerate |
                                ::ranges::views::filter(
                                    [&sampled](auto&& p) -> bool
                                    {
                                        return sampled.contains(p.first);
                                    }) |
                                ::ranges::views::values | ::ranges::views::values)
        {
            mask[i] = true;
        }
        return mask;
    }

    float HeuristicScenarioSelector::label(
        const std::shared_ptr<EuclideanGraphEnvironment>& environment,
        std::vector<std::pair<unsigned int, unsigned int>> task_edges,
        std::vector<std::pair<unsigned int, unsigned int>> precedence_transition_edges,
        std::vector<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>> mutex_transition_edges) const
    {
        float rv = 0.0f;
        for(auto [task_nr, edge_ids]: task_edges | ::ranges::view::enumerate)
        {
            rv += m_problem_inputs->planTask(task_nr)->staticDuration();
            const auto [i, j] = edge_ids;

            // No travel during task
            if(i == j)
            {
                continue;
            }

            float speed = std::numeric_limits<float>::max();
            for(auto robot: m_problem_inputs->coalition(task_nr))
            {
                speed = std::min(speed, robot->speed());
            }

            // Weight by number of robots?
            rv += environment->findEdge(i, j)->cost() / speed;
        }

        //        for(auto [i, j]: m_precedence_transition_edges)
        //        {
        //            rv += environment->findPossibleEdge(i, j)->cost();
        //        }
        //
        //        for(auto [i, j, k, l]: m_mutex_trasition_edges)
        //        {
        //            // weight by number of robots?
        //            const float i_to_j_cost = environment->findPossibleEdge(i, j)->cost();
        //            const float j_to_i_cost = environment->findPossibleEdge(k, l)->cost();
        //            rv += 0.5 * i_to_j_cost + 0.5 * j_to_i_cost;
        //        }

        return rv;
    }
}  // namespace grstapse