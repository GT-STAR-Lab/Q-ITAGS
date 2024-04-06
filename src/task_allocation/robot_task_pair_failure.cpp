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
#include "grstapse/task_allocation/robot_task_pair_failure.hpp"

// Global
#include <utility>

namespace grstapse
{
    RobotTaskPairFailure::RobotTaskPairFailure(const detail::RobotTaskPairFailureParametersImpl& parameters)
        : robot(parameters.robot)
        , task_i(parameters.task_i)
        , task_j(parameters.task_j)
    {}

    RobotTaskPairFailure::RobotTaskPairFailure(detail::RobotTaskPairFailureParametersImpl&& parameters)
        : robot(std::move(parameters.robot))
        , task_i(std::move(parameters.task_i))
        , task_j(std::move(parameters.task_j))
    {}

    RobotTaskPairFailure::RobotTaskPairFailure(unsigned int robot, unsigned int task_i, unsigned int task_j)
        : robot(robot)
        , task_i(task_i)
        , task_j(task_j)
    {}
}  // namespace grstapse