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
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/scenario_selector_base.hpp"

// Local
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    ScenarioSelectorBase::ScenarioSelectorBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    std::pair<unsigned int, unsigned int> ScenarioSelectorBase::getEdge(const std::shared_ptr<const Task>& i,
                                                                        const std::shared_ptr<const Task>& j)
    {
        return {std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(i->terminalConfiguration())->id(),
                std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(j->initialConfiguration())->id()};
    }

    std::pair<unsigned int, unsigned int> ScenarioSelectorBase::getEdge(const std::shared_ptr<const Task>& task)
    {
        return {std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(task->initialConfiguration())->id(),
                std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(task->terminalConfiguration())->id()};
    }

    std::pair<unsigned int, unsigned int> ScenarioSelectorBase::getEdge(unsigned int i, unsigned int j) const
    {
        return getEdge(m_problem_inputs->planTask(i), m_problem_inputs->planTask(j));
    }

    std::pair<unsigned int, unsigned int> ScenarioSelectorBase::getEdge(unsigned int i) const
    {
        return getEdge(m_problem_inputs->planTask(i));
    }
}  // namespace grstapse