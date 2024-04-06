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
#ifndef NO_MILP

// Global
#    include <fstream>
// External
#    include <fmt/format.h>
#    include <gtest/gtest.h>
// Project
#    include <grstapse/common/utilities/json_extension.hpp>
#    include <grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp>
#    include <grstapse/scheduling/scheduler_result.hpp>
// Local
#    include "mock_normalized_schedule_quality.hpp"
#    include "scheduling_setup.hpp"

namespace grstapse::unittests
{
    /*!
     * Test that a full run of scheduler works
     */
    TEST(DeterministicMilpScheduler, FullRun)
    {
        auto run_test = [](const std::string& identifier,
                           const PlanOption plan_option,
                           const AllocationOption allocation_option,
                           const bool homogeneous,
                           const std::vector<std::pair<float, float>>& correct_timepoints,
                           const float correct_makespan)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            DeterministicMilpScheduler scheduler(scheduler_problem_inputs);

            std::shared_ptr<const SchedulerResult> result = scheduler.solve();
            ASSERT_TRUE(result->success());

            auto schedule = std::dynamic_pointer_cast<const DeterministicSchedule>(result->schedule());
            ASSERT_TRUE(schedule);

            ASSERT_NEAR(schedule->makespan(), correct_makespan, 1e-2)
                << fmt::format("{0:s}: Incorrect makespan (true: {1:f}; computed: {2:f})",
                               identifier,
                               correct_makespan,
                               schedule->makespan());

            const auto& timepoints = schedule->timepoints();
            for(unsigned int i = 0, end = timepoints.size(); i < end; ++i)
            {
                ASSERT_NEAR(timepoints[i].first, correct_timepoints[i].first, 1e-4)
                    << fmt::format("{0:s}: Incorrect start timepoint for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_timepoints[i].first,
                                   timepoints[i].first);
                ASSERT_NEAR(timepoints[i].second, correct_timepoints[i].second, 1e-4)
                    << fmt::format("{0:s}: Incorrect start timepoint for task {1:d} (true: {2:f}; computed: {3:f})",
                                   identifier,
                                   i,
                                   correct_timepoints[i].second,
                                   timepoints[i].second);
            }
        };

        run_test("TO-I",
                 PlanOption::e_total_order,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {13.0f, 29.0f}},
                 29.0f);
        run_test("Branch-I",
                 PlanOption::e_branch,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {6.0f, 22.0f}},
                 22.0f);
        // Robot 1 does tasks 1 and 3 (tests transition)
        run_test("Branch-MR",
                 PlanOption::e_branch,
                 AllocationOption::e_multi_task_robot,
                 true,
                 {{5.0f, 6.0f}, {6.0f, 13.0f}, {16.0f, 32.0f}},
                 32.0f);
        // If this passes, then the scheduler iteration works
        run_test("Complex 2",
                 PlanOption::e_complex,
                 AllocationOption::e_complex2,
                 false,
                 {{4.1667f, 5.1667f},
                  {38.3339f, 45.3339f},
                  {25.8339f, 39.3339f},
                  {56.5142f, 58.5142f},
                  {58.5142f, 66.2283f},
                  {20.1137f, 25.8339f},
                  {72.9266f, 87.4020f}},
                 87.4020f);
        MilpSolverBase::clearEnvironments();
    }

}  // namespace grstapse::unittests
#endif