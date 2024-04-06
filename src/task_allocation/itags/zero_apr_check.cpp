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
#include "grstapse/task_allocation/itags/zero_apr_check.hpp"

// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    ZeroAprCheck::ZeroAprCheck(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    bool ZeroAprCheck::operator()(const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const
    {
        // A
        const Eigen::MatrixXf& allocation = node->allocation();

        // E
        Eigen::MatrixXf traits_mismatch_matrix = traitsMismatchMatrix(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                                      allocation,
                                                                      m_problem_inputs->desiredTraitsMatrix(),
                                                                      m_problem_inputs->teamTraitsMatrix());

        // Any positive value means that there are traits that are unsatisfied
        return !(traits_mismatch_matrix.array() > 0).any();
    }
}  // namespace grstapse