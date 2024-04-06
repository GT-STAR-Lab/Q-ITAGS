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
     * Evaluates an allocation by computing the quality using a defined quality model
     *
     * \see Itags
     *
     */
    class NormalizedAllocationQuality : public HeuristicBase<IncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        NormalizedAllocationQuality(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

        //! \returns The percentage of the desired traits left unsatisfied by the allocation in \p node
        [[nodiscard]] float operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const final override;

        // function for setting the normalization variables
        void setMaxAndMinQuality(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        float m_max_quality;
        float m_min_quality;
    };
}  // namespace grstapse