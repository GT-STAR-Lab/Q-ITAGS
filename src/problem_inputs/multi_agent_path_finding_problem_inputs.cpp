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
#include "grstapse/problem_inputs/multi_agent_path_finding_problem_inputs.hpp"

namespace grstapse
{
    MultiAgentPathFindingProblemInputs::MultiAgentPathFindingProblemInputs(
        const std::shared_ptr<const GridMap>& map,
        const std::vector<std::shared_ptr<const GridCell>>& initial_states,
        const std::vector<std::shared_ptr<const GridCell>>& goal_states)
        : m_map(map)
        , m_initial_states(initial_states)
        , m_goal_states(goal_states)
    {
        assert(m_initial_states.size() == m_goal_states.size());
    }
}  // namespace grstapse