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

// Global
#include <memory>
#include <vector>

namespace grstapse
{
    // Forward Declarations
    class Task;
    class Robot;
    class ConfigurationBase;

    /*!
     * Abstract base class for interfacing between the scheduler and motion planner
     */
    class SchedulerMotionPlannerInterfaceBase
    {
       public:
        //! \returns How long \p coalition will take to accomplish \p task_nr'th task
        [[nodiscard]] virtual float computeTaskDuration(
            const std::shared_ptr<const Task>& task,
            const std::vector<std::shared_ptr<const Robot>>& coalition) const = 0;

        /*!
         * \returns Whether the motion plan from a \p robot's initial configuration to a specific \p configuration has
         * already been computed
         */
        [[nodiscard]] virtual bool isInitialTransitionMemoized(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const = 0;

        /*!
         * \returns The time it will take a \p robot to transition from it initial configuration to \p configuration
         */
        [[nodiscard]] virtual float computeInitialTransitionDuration(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const = 0;

        /*!
         * \returns An estimate of the time it will take a \p robot to transition from it initial configuration to \p
         * configuration
         */
        [[nodiscard]] virtual float computeInitialTransitionDurationHeuristic(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const;

        /*!
         * \returns Whether the motion plan from \p initial to \p goal has already been computed for \p robot
         */
        [[nodiscard]] virtual bool isTransitionMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                                                        const std::shared_ptr<const ConfigurationBase>& goal,
                                                        const std::shared_ptr<const Robot>& robot) const = 0;

        /*!
         * \returns The time it will take a \p robot to transition from \p initial to \p goal
         */
        [[nodiscard]] virtual float computeTransitionDuration(const std::shared_ptr<const ConfigurationBase>& initial,
                                                              const std::shared_ptr<const ConfigurationBase>& goal,
                                                              const std::shared_ptr<const Robot>& robot) const = 0;

        /*!
         * \returns An estimate of the time it will take a \p robot to transition from \p initial to \p goal
         */
        [[nodiscard]] virtual float computeTransitionDurationHeuristic(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& goal,
            const std::shared_ptr<const Robot>& robot) const;
    };
}  // namespace grstapse