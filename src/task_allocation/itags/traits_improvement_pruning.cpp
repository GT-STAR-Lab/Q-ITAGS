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
#include "grstapse/task_allocation/itags/traits_improvement_pruning.hpp"

// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    TraitsImprovementPruning::TraitsImprovementPruning(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    bool TraitsImprovementPruning::operator()(const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const
    {
        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        Eigen::MatrixXf potential_successor_matrix = node->allocation();

        Eigen::MatrixXf parent_matrix                                            = potential_successor_matrix;
        parent_matrix(node->lastAssigment()->task, node->lastAssigment()->robot) = 0.0f;

        const float potential_successor_error = traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                                    potential_successor_matrix,
                                                                    m_problem_inputs->desiredTraitsMatrix(),
                                                                    m_problem_inputs->teamTraitsMatrix());

        const float parent_error = traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                       parent_matrix,
                                                       m_problem_inputs->desiredTraitsMatrix(),
                                                       m_problem_inputs->teamTraitsMatrix());

        return potential_successor_error >= parent_error;
    }
}  // namespace grstapse