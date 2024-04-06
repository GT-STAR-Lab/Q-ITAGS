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
#include <experimental/source_location>
#include <stdexcept>
#include <string>

// External
#include <fmt/format.h>

// Local
#include "grstapse/common/utilities/logger.hpp"

/*!
 * \brief A collection of utility functions for error/exceptions
 *
 * \file error.hpp
 *
 * \todo(Andrew): have an option that is not preformatted
 */

namespace grstapse
{
    /*!
     * \brief
     *
     * \param formatted_message
     * \param location
     *
     * \returns
     */
    std::logic_error createLogicError(
        const std::string_view& formatted_message,
        const std::experimental::source_location& location = std::experimental::source_location::current());

    /*!
     * \brief
     *
     * \param formatted_message
     * \param location
     *
     * \returns
     */
    std::runtime_error createRuntimeError(
        const std::string_view& formatted_message,
        const std::experimental::source_location location = std::experimental::source_location::current());
}  // namespace grstapse