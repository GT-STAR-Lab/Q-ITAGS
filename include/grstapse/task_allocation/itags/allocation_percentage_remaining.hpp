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
#pragma once

// Local
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /*!
     * Evaluates an allocation by computing the percentage of the desired traits left unsatisfied
     *
     * \see Itags
     *
     * \cite Neville, G., Messing, A., Ravichandar, H., Hutchinson, S., & Chernova, S. (2021, August). An interleaved
     *       approach to trait-based task allocation and scheduling. In 2021 IEEE/RSJ International Conference on
     *       Intelligent Robots and Systems (IROS) (pp. 1507-1514). IEEE.
     */
    class AllocationPercentageRemaining : public HeuristicBase<IncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        AllocationPercentageRemaining(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

        //! \returns The percentage of the desired traits left unsatisfied by the allocation in \p node
        [[nodiscard]] float operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const final override;

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        float m_desired_traits_sum;
    };
}  // namespace grstapse