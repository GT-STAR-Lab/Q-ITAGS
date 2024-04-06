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
#include "grstapse/task_allocation/itags/allocation_percentage_remaining.hpp"

// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    AllocationPercentageRemaining::AllocationPercentageRemaining(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
        , m_desired_traits_sum(problem_inputs->desiredTraitsMatrix().sum())
    {}

    float AllocationPercentageRemaining::operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const
    {
        const Eigen::MatrixXf& allocation = node->allocation();
        const float traits_mismatch_error = traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                                allocation,
                                                                m_problem_inputs->desiredTraitsMatrix(),
                                                                m_problem_inputs->teamTraitsMatrix());

        // ||max(E(A), 0)||_{1, 1} / ||Y||_{1,1}
        return traits_mismatch_error / m_desired_traits_sum;
    }
}  // namespace grstapse