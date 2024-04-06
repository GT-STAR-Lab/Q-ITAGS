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

// Global
#include <map>
// Local
#include "grstapse/common/search/pruning_method_base.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

namespace grstapse
{
    // Forward Declaration
    class FailureReason;
    class ItagsProblemInputs;

    /*!
     * A pruning method that used prior failure reasons from scheduling/motion planning to prune future node
     */
    class ItagsPreviousFailurePruningMethod : public PruningMethodBase<IncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        explicit ItagsPreviousFailurePruningMethod(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs);

        //! \copydoc PruningMethodBase
        bool operator()(const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const override;

        //! Adds a failure reason to use for future pruning
        void addFailureReason(const std::shared_ptr<const FailureReason>& failure_reason);

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        std::multimap<unsigned int, unsigned int> m_robot_task_failures;  //!< key is the robot, value is the task
        std::multimap<unsigned int, std::pair<unsigned int, unsigned int>>
            m_robot_task_pair_failures;  //!< key is the robot, value is the pair of tasks
        std::multimap<std::string, unsigned int> m_species_task_failures;
        std::multimap<std::string, std::pair<unsigned int, unsigned int>> m_species_task_pair_failures;
    };

}  // namespace grstapse