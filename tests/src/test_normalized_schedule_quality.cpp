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
// Global
#include <memory>
// External
#include <Eigen/Core>
#include <fmt/format.h>
#include <gtest/gtest.h>
// Project
#include <grstapse/problem_inputs/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/incremental_task_allocation_node.hpp>
// Local
#include "mock_itags_problem_inputs.hpp"
#include "mock_normalized_schedule_quality.hpp"

namespace grstapse::unittests
{
    /*!
     * Tests the basic NSQ equation (sched - sched_best) / (sched_worst - sched_best)
     */
    TEST(NormalizedScheduleQuality, EquationCheck)
    {
        // Empty problem inputs
        auto itags_problem_inputs =
            std::shared_ptr<mocks::MockItagsProblemInputs>(new mocks::MockItagsProblemInputs(nullptr, {}, 0.0f, 2.0f));
        itags_problem_inputs->validate();

        auto nsq = std::make_shared<mocks::MockNormalizedScheduleQuality>(itags_problem_inputs, 1.0f);

        // Empty node
        auto node = std::make_shared<IncrementalTaskAllocationNode>(MatrixDimensions{0, 0});

        const float result = nsq->operator()(node);
        ASSERT_FLOAT_EQ(result, 0.5f) << fmt::format("NSQ computation incorrect (true: {0:f}; computed: {1:f})",
                                                     0.5f,
                                                     result);
    }
}  // namespace grstapse::unittests