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
#include "grstapse/problem_generators/itags_problem_generator.hpp"

// External
#include <range/v3/all.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/std_extension.hpp"

namespace grstapse
{
    nlohmann::json ItagsProblemGenerator::generate() const
    {
        nlohmann::json j = GrstapsProblemGenerator::generate();

        j[constants::k_precedence_constraints] = generatePrecedenceConstraints();

        const int num_tasks               = j[constants::k_tasks].size();
        j[constants::k_plan_task_indices] = ranges::views::iota(0, num_tasks) | ranges::to<std::vector<int>>();

        return j;
    }
}  // namespace grstapse