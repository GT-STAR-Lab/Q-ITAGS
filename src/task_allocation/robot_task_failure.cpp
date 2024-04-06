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
#include "grstapse/task_allocation/robot_task_failure.hpp"

// Global
#include <utility>

namespace grstapse
{
    RobotTaskFailure::RobotTaskFailure(const detail::RobotTaskFailureParametersImpl& parameters)
        : robot(parameters.robot)
        , task(parameters.task)
    {}

    RobotTaskFailure::RobotTaskFailure(detail::RobotTaskFailureParametersImpl&& parameters)
        : robot(std::move(parameters.robot))
        , task(std::move(parameters.task))
    {}

    RobotTaskFailure::RobotTaskFailure(unsigned int robot, unsigned int task)
        : robot(robot)
        , task(task)
    {}
}  // namespace grstapse