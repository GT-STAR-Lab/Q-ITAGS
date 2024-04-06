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
#include "grstapse/common/search/successor_generator_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    /*!
     * Generates nodes that expand a task allocation node by adding a single robot to a single task
     */
    class IncrementalAllocationGenerator : public SuccessorGeneratorBase<IncrementalTaskAllocationNode>
    {
        using Base_ = SuccessorGeneratorBase<IncrementalTaskAllocationNode>;

       public:
        /*!
         * \brief Constructor
         *
         * \param parameters
         */
        explicit IncrementalAllocationGenerator(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

       private:
        //! \returns Whether the node is valid
        [[nodiscard]] bool isValidNode(
            const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const final override;

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };
}  // namespace grstapse