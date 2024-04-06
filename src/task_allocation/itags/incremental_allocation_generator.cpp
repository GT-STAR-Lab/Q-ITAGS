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
#include "grstapse/task_allocation/itags/incremental_allocation_generator.hpp"

// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/incremental_allocation_edge_applier.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    IncrementalAllocationGenerator::IncrementalAllocationGenerator(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {
        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        const unsigned int number_of_robots = m_problem_inputs->numberOfRobots();
        const unsigned int number_of_tasks  = m_problem_inputs->numberOfPlanTasks();

        std::vector<std::shared_ptr<const typename Base_::EdgeApplier>> edge_appliers;
        edge_appliers.reserve(number_of_tasks * number_of_robots);

        for(unsigned int m = 0; m < number_of_tasks; ++m)
        {
            for(unsigned int n = 0; n < number_of_robots; ++n)
            {
                edge_appliers.push_back(
                    std::make_shared<const IncrementalAllocationEdgeApplier>(Assignment{.task = m, .robot = n},
                                                                             problem_inputs,
                                                                             m_problem_inputs->useReverse()));
            }
        }
        setEdgeAppliers(edge_appliers);
    }

    bool IncrementalAllocationGenerator::isValidNode(
        const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const
    {
        return true;
    }
}  // namespace grstapse