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
#include <grstapse/problem_inputs/itags_problem_inputs.hpp>
#include <grstapse/problem_inputs/scheduler_problem_inputs.hpp>
#include <grstapse/scheduling/milp/stochastic/benders/benders_stochastic_milp_scheduler.hpp>

namespace grstapse::unittests
{
// Deprecated
#if 0
    TEST(BendersStochasticMilpScheduler, Simple)
    {
        std::ifstream in("data/task_allocation/itags_problem_inputs/itags_polypixel_10maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 10, 5> allocation;
        allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        BendersStochasticMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule = scheduler.solve();
    }
#endif
}  // namespace grstapse::unittests