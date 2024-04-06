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
#include "grstapse/task_allocation/itags/itags_previous_failure_pruning_method.hpp"

// Local
#include "grstapse/common/utilities/compound_failure_reason.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task_allocation/robot_task_failure.hpp"
#include "grstapse/task_allocation/robot_task_pair_failure.hpp"
#include "grstapse/task_allocation/species_task_failure.hpp"
#include "grstapse/task_allocation/species_task_pair_failure.hpp"

namespace grstapse
{
    ItagsPreviousFailurePruningMethod::ItagsPreviousFailurePruningMethod(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    bool ItagsPreviousFailurePruningMethod::operator()(
        const std::shared_ptr<const IncrementalTaskAllocationNode>& node) const
    {
        std::optional<Assignment> assignment = node->lastAssigment();
        // Should only happen for the root which this should get called on
        if(!assignment.has_value())
        {
            return false;
        }

        const unsigned int robot = assignment.value().robot;
        const unsigned int task  = assignment.value().task;

        if(m_robot_task_failures.contains(robot))
        {
            auto begin_end = m_robot_task_failures.equal_range(robot);
            for(auto iter = begin_end.first; iter != begin_end.second; ++iter)
            {
                if(iter->second == task)
                {
                    return true;
                }
            }
        }

        if(m_robot_task_pair_failures.contains(robot))
        {
            Eigen::MatrixXf allocation = node->allocation();
            auto begin_end             = m_robot_task_pair_failures.equal_range(robot);
            for(auto iter = begin_end.first; iter != begin_end.second; ++iter)
            {
                const std::pair<unsigned int, unsigned int>& p = iter->second;
                if(p.first == task && allocation(p.second, robot))
                {
                    return true;
                }

                else if(p.second == task && allocation(p.first, robot))
                {
                    return true;
                }
            }
        }

        const std::string& species = m_problem_inputs->robot(robot)->species()->name();

        if(m_species_task_failures.contains(species))
        {
            auto begin_end = m_species_task_failures.equal_range(species);
            for(auto iter = begin_end.first; iter != begin_end.second; ++iter)
            {
                if(iter->second == task)
                {
                    return true;
                }
            }
        }

        if(m_species_task_pair_failures.contains(species))
        {
            Eigen::MatrixXf allocation = node->allocation();
            auto begin_end             = m_species_task_pair_failures.equal_range(species);
            for(auto iter = begin_end.first; iter != begin_end.second; ++iter)
            {
                const std::pair<unsigned int, unsigned int>& p = iter->second;
                if(p.first == task && allocation(p.second, robot))
                {
                    return true;
                }

                else if(p.second == task && allocation(p.first, robot))
                {
                    return true;
                }
            }
        }

        return false;
    }

    void ItagsPreviousFailurePruningMethod::addFailureReason(const std::shared_ptr<const FailureReason>& failure_reason)
    {
        if(auto robot_task_failure = std::dynamic_pointer_cast<const RobotTaskFailure>(failure_reason);
           robot_task_failure)
        {
            m_robot_task_failures.emplace(robot_task_failure->robot, robot_task_failure->task);
            return;
        }

        if(auto robot_task_pair_failure = std::dynamic_pointer_cast<const RobotTaskPairFailure>(failure_reason);
           robot_task_pair_failure)
        {
            m_robot_task_pair_failures.emplace(
                robot_task_pair_failure->robot,
                std::pair(robot_task_pair_failure->task_i, robot_task_pair_failure->task_j));
            return;
        }

        if(auto species_task_failure = std::dynamic_pointer_cast<const SpeciesTaskFailure>(failure_reason);
           species_task_failure)
        {
            m_species_task_failures.emplace(species_task_failure->species, species_task_failure->task);
            return;
        }

        if(auto species_task_pair_failure = std::dynamic_pointer_cast<const SpeciesTaskPairFailure>(failure_reason);
           species_task_pair_failure)
        {
            m_species_task_pair_failures.emplace(species_task_pair_failure->species,
                                                 std::pair(species_task_pair_failure->predecessor_task_index,
                                                           species_task_pair_failure->successor_task_index));
            return;
        }

        if(auto compound_failure = std::dynamic_pointer_cast<const CompoundFailureReason>(failure_reason);
           compound_failure)
        {
            for(const std::shared_ptr<const FailureReason>& inner_reason: compound_failure->reasons())
            {
                addFailureReason(inner_reason);
            }
            return;
        }

        throw createLogicError("Unknown failure reason");
    }
}  // namespace grstapse