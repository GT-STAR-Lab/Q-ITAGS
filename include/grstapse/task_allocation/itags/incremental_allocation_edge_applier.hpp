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
#include "grstapse/common/search/edge_applier_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /*!
     * Functor to add an edge representing the allocation of a specific robot to a specific task
     */
    class IncrementalAllocationEdgeApplier : public EdgeApplierBase<IncrementalTaskAllocationNode>
    {
       public:
        /*!
         * Constructor
         *
         * \param assignment The assignment this edge represents
         * \param problem_inputs Inputs to the task allocation problem
         */
        IncrementalAllocationEdgeApplier(const Assignment& assignment,
                                         const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                         bool use_reverse = false);

        //! \returns Whether the edge this edge applier represents can be added to \p base
        [[nodiscard]] bool isApplicable(
            const std::shared_ptr<const IncrementalTaskAllocationNode>& base) const final override;

        //! \returns The succeeding node if this edge applier can be applied, nullptr otherwise
        [[nodiscard]] std::shared_ptr<IncrementalTaskAllocationNode> apply(
            const std::shared_ptr<const IncrementalTaskAllocationNode>& base) const final override;

       private:
        Assignment m_assignment;
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        bool m_use_reverse;
    };
}  // namespace grstapse