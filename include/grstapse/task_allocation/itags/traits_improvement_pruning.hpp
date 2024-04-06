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
#include "grstapse/common/search/pruning_method_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

namespace grstapse
{
    // Forward Declarations
    class ItagsProblemInputs;

    /*!
     *  Prunes a node if it does not improve the traits mismatch error with respect to its parent node
     */
    class TraitsImprovementPruning : public PruningMethodBase<IncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        explicit TraitsImprovementPruning(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

        //! \copydoc PruningMethodBase
        [[nodiscard]] virtual bool operator()(
            const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const final override;

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
    };

}  // namespace grstapse