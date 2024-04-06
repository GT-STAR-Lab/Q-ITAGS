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
#include "grstapse/task_allocation/itags/incremental_allocation_edge_applier.hpp"

// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    IncrementalAllocationEdgeApplier::IncrementalAllocationEdgeApplier(
        const Assignment& assignment,
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        bool use_reverse)
        : m_assignment(assignment)
        , m_problem_inputs(problem_inputs)
        , m_use_reverse(use_reverse)
    {}

    bool IncrementalAllocationEdgeApplier::isApplicable(
        const std::shared_ptr<const IncrementalTaskAllocationNode>& base) const
    {
        // If the assignment has already been added then ignore
        std::shared_ptr<const IncrementalTaskAllocationNode> parent;
        for(parent = base; parent != nullptr; parent = parent->parent())
        {
            if(const std::optional<Assignment>& last_assignment = parent->lastAssigment();
               last_assignment.has_value() && last_assignment.value() == m_assignment)
            {
                return false;
            }
        }

        return true;
    }

    std::shared_ptr<IncrementalTaskAllocationNode> IncrementalAllocationEdgeApplier::apply(
        const std::shared_ptr<const IncrementalTaskAllocationNode>& base) const
    {
        return std::make_shared<IncrementalTaskAllocationNode>(m_assignment, base, m_use_reverse);
    }
}  // namespace grstapse