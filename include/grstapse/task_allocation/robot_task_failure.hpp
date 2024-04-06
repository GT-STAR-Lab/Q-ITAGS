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
#include <cstdint>
// Local
#include "grstapse/common/utilities/failure_reason.hpp"

namespace grstapse
{
    namespace detail
    {
        /*!
         * \brief Hack for designated initializers
         */
        struct RobotTaskFailureParametersImpl
        {
            unsigned int robot;
            unsigned int task;
        };
    }  // namespace detail

    /*!
     * \brief Failure that represents that a specific robot cannot be assigned to a specific task
     */
    struct RobotTaskFailure : public FailureReason
    {
        unsigned int robot;
        unsigned int task;

       protected:
        /*!
         * \brief Constructor
         *
         * \param parameters
         */
        explicit RobotTaskFailure(const detail::RobotTaskFailureParametersImpl& parameters);

        /*!
         * \brief Constructor
         *
         * \param parameters
         */
        RobotTaskFailure(detail::RobotTaskFailureParametersImpl&& parameters);

        /*!
         * \brief Constructor
         *
         * \param robot
         * \param task
         */
        RobotTaskFailure(unsigned int robot, unsigned int task);
    };
}  // namespace grstapse