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

// Project
#include <grstapse/common/utilities/std_extension.hpp>
#include <grstapse/problem_inputs/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/task_allocation_math.hpp>
// External
#include <range/v3/range/conversion.hpp>
// Local
#include "mock_grstaps_problem_inputs.hpp"

namespace grstapse::mocks
{
    /*!
     * Mock to interface with some of the private functions for testing
     */
    class MockItagsProblemInputs : public ItagsProblemInputs
    {
       public:
        //! Default Constructor
        MockItagsProblemInputs()
            : ItagsProblemInputs(ProblemInputs::s_this_is_protected_tag)
        {}

        //! Constructor
        MockItagsProblemInputs(const std::shared_ptr<const MockGrstapsProblemInputs>& grstaps_problem_inputs,
                               const std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints = {},
                               const float schedule_best_makespan                                            = 0.0f,
                               const float schedule_worst_makespan = std::numeric_limits<float>::infinity())
            : ItagsProblemInputs(ProblemInputs::s_this_is_protected_tag)
        {
            m_grstaps_problem_inputs = grstaps_problem_inputs;
            if(m_grstaps_problem_inputs != nullptr)
            {
                m_plan_task_indices = ::ranges::views::iota(0u, m_grstaps_problem_inputs->numberOfTasks()) |
                                      ::ranges::to<std::vector<unsigned int>>();
                m_desired_traits_matrix = grstapse::desiredTraitsMatrix(
                    planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
            }
            m_precedence_constraints  = precedence_constraints;
            m_precedence_constraints  = addPrecedenceTransitiveConstraints(precedence_constraints);
            m_schedule_best_makespan  = schedule_best_makespan;
            m_schedule_worst_makespan = schedule_worst_makespan;
        }

        void setDesiredTraitsMatrix(const Eigen::MatrixXf& matrix)
        {
            m_desired_traits_matrix = matrix;
        }

        using ItagsProblemInputs::loadTasks;
    };

}  // namespace grstapse::mocks