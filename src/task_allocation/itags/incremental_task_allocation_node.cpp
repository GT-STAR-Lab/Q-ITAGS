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
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/hash_extension.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/geometric_planning/query_results/motion_planner_query_result_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/normalized_schedule_quality.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    unsigned int IncrementalTaskAllocationNode::s_next_id = 0;

    IncrementalTaskAllocationNode::IncrementalTaskAllocationNode(const MatrixDimensions& dimensions, bool use_reverse)
        : Base_(s_next_id++, nullptr)
        , m_last_assigment(std::nullopt)
        , m_matrix_dimensions(dimensions)
        , m_schedule(nullptr)
        , m_use_reverse(use_reverse)
    {}

    IncrementalTaskAllocationNode::IncrementalTaskAllocationNode(
        const Assignment& assignment,
        const std::shared_ptr<const IncrementalTaskAllocationNode>& parent,
        bool use_reverse)
        : Base_(s_next_id++, parent)
        , m_last_assigment(assignment)
        , m_matrix_dimensions(std::nullopt)
        , m_schedule(nullptr)
        , m_use_reverse(use_reverse)
    {
        assert(parent);
    }

    const MatrixDimensions& IncrementalTaskAllocationNode::matrixDimensions() const
    {
        if(m_matrix_dimensions.has_value())
        {
            return m_matrix_dimensions.value();
        }
        return m_parent->matrixDimensions();
    }

    Eigen::MatrixXf IncrementalTaskAllocationNode::allocation() const
    {
        if(!m_use_reverse)
        {
            const MatrixDimensions& dimensions = matrixDimensions();

            // Allocation matrix is M X N (number_of_tasks X number_of_robots)
            Eigen::MatrixXf matrix(dimensions.height, dimensions.width);  // (# rows, # columns)
            matrix.setZero();
            if(m_last_assigment == std::nullopt)
            {
                return matrix;
            }

            matrix(m_last_assigment->task, m_last_assigment->robot) = 1.0f;

            std::shared_ptr<const IncrementalTaskAllocationNode> parent;
            for(parent = m_parent; parent->m_last_assigment != std::nullopt; parent = parent->parent())
            {
                matrix(parent->m_last_assigment->task, parent->m_last_assigment->robot) = 1.0f;
            }
            return matrix;
        }
        else
        {
            {
                const MatrixDimensions& dimensions = matrixDimensions();

                // Allocation matrix is M X N (number_of_tasks X number_of_robots)
                Eigen::MatrixXf matrix = Eigen::MatrixXf::Ones(dimensions.height, dimensions.width);
                if(m_last_assigment == std::nullopt)
                {
                    return matrix;
                }

                matrix(m_last_assigment->task, m_last_assigment->robot) = 0.0f;

                std::shared_ptr<const IncrementalTaskAllocationNode> parent;
                for(parent = m_parent; parent->m_last_assigment != std::nullopt; parent = parent->parent())
                {
                    matrix(parent->m_last_assigment->task, parent->m_last_assigment->robot) = 0.0f;
                }
                return matrix;
            }
        }
    }

    unsigned int IncrementalTaskAllocationNode::hash() const
    {
        return std::hash<Eigen::MatrixXf>()(allocation());
    }

    nlohmann::json IncrementalTaskAllocationNode::serializeToJson(
        const std::shared_ptr<const ProblemInputs>& problem_inputs) const
    {
        if(m_schedule)
        {
            return m_schedule->serializeToJson(std::make_shared<SchedulerProblemInputs>(
                std::dynamic_pointer_cast<const ItagsProblemInputs>(problem_inputs),
                allocation()));
        }

        // No schedule already computed
        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(
            std::dynamic_pointer_cast<const ItagsProblemInputs>(problem_inputs),
            allocation());
        DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
        std::shared_ptr<const SchedulerResult> result = scheduler.solve();
        if(result->success())
        {
            return result->schedule()->serializeToJson(scheduler_problem_inputs);
        }

        // Creates null json
        return nullptr;
    }
}  // namespace grstapse