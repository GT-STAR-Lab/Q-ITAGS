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
#include <fstream>
// External
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
// Project
#include <grstapse/common/utilities/constants.hpp>
#include <grstapse/common/utilities/time_keeper.hpp>
#include <grstapse/config.hpp>
#include <grstapse/problem_inputs/itags_problem_inputs.hpp>
#include <grstapse/problem_inputs/scheduler_problem_inputs.hpp>
#include <grstapse/scheduling/milp/stochastic/heuristic_approximation/heuristic_approximation_stochastic_scheduler.hpp>
#include <grstapse/scheduling/schedule_base.hpp>
#include <grstapse/scheduling/scheduler_result.hpp>

namespace grstapse::unittests
{
    TEST(HeuristicApproximationStochasticScheduler, Simple)
    {
        {
            std::ifstream in(
                std::string(s_data_dir) +
                std::string("/problem_inputs/itags/itags_heuristic_polypixel_400maps_10tasks_5robots.json"));
            nlohmann::json j;
            in >> j;
            auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

            Eigen::Matrix<float, 10, 5> allocation;
            allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
                1.0f;

            auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

            HeuristicApproximationStochasticScheduler scheduler(scheduler_problem_inputs);
            auto schedule        = scheduler.solve();
            const float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
            const float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
            const float s_time   = smp_time - mp_time;
            fmt::print("\tHA:\n\t\tTime: {0:0.3f}s\n\t\tMakespan: {1:0.3f}\n",
                       s_time,
                       schedule->schedule()->makespan());
        }
        MilpSolverBase::clearEnvironments();
    }
}  // namespace grstapse::unittests