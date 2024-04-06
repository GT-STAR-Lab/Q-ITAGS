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
#include <cstddef>
// Local
#include "grstapse/common/utilities/failure_reason.hpp"

namespace grstapse
{
    namespace detail
    {
        /*!
         * \brief Hack for designated initializers
         */
        struct RobotTaskPairFailureParametersImpl
        {
            unsigned int robot;
            unsigned int task_i;
            unsigned int task_j;
        };
    }  // namespace detail

    /*!
     * \brief Failure that represents that a specific robot cannot be assigned to both task's i and j
     */
    struct RobotTaskPairFailure : public FailureReason
    {
        unsigned int robot;
        unsigned int task_i;
        unsigned int task_j;

       protected:
        /*!
         * \brief Constructor
         *
         * \param parameters
         */
        explicit RobotTaskPairFailure(const detail::RobotTaskPairFailureParametersImpl& parameters);

        /*!
         * \brief Constructor
         *
         * \param parameters
         */
        RobotTaskPairFailure(detail::RobotTaskPairFailureParametersImpl&& parameters);

        /*!
         * \brief Constructor
         *
         * \param robot
         * \param task_i
         * \param task_j
         */
        RobotTaskPairFailure(unsigned int robot, unsigned int task_i, unsigned int task_j);
    };
}  // namespace grstapse