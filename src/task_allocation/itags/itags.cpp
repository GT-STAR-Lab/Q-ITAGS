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
#include "grstapse/task_allocation/itags/itags.hpp"

namespace grstapse
{
    Itags::Itags(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        const std::shared_ptr<const HeuristicBase<IncrementalTaskAllocationNode>>& heuristic,
        const std::shared_ptr<const SuccessorGeneratorBase<IncrementalTaskAllocationNode>>& successor_generator,
        const std::shared_ptr<const GoalCheckBase<IncrementalTaskAllocationNode>>& goal_check,
        const std::shared_ptr<const MemoizationBase<IncrementalTaskAllocationNode>>& memoization,
        const std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>>& pre_pruning_method,
        const std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>>& post_pruning_method,
        bool use_reverse)
        : Base_{problem_inputs->itagsParameters(),
                {.heuristic           = heuristic,
                 .successor_generator = successor_generator,
                 .goal_check          = goal_check,
                 .memoization         = memoization,
                 .prepruning_method   = pre_pruning_method,
                 .postpruning_method  = post_pruning_method}}
        , m_problem_inputs(problem_inputs)
        , m_use_reverse(use_reverse)
    {}

    bool Itags::isAllocatable() const
    {
        if(!m_use_reverse)
        {
            // N
            const unsigned int num_robots = m_problem_inputs->numberOfRobots();
            // M
            const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();

            // A \in \R^{M \times N}
            Eigen::MatrixXf allocation = Eigen::MatrixXf::Ones(num_tasks, num_robots);

            return traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                       allocation,
                                       m_problem_inputs->desiredTraitsMatrix(),
                                       m_problem_inputs->teamTraitsMatrix()) == 0;
        }
        else
        {
            return true;
        }
    }

    std::shared_ptr<IncrementalTaskAllocationNode> Itags::createRootNode()
    {
        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        const unsigned int num_tasks  = m_problem_inputs->numberOfPlanTasks();
        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        return std::make_shared<IncrementalTaskAllocationNode>(
            MatrixDimensions{.height = num_tasks, .width = num_robots},
            m_use_reverse);
    }
}  // namespace grstapse