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
#include <string>
// Local
#include "grstapse/common/utilities/failure_reason.hpp"

namespace grstapse
{
    namespace detail
    {
        //! Hack for designated initializers
        struct SpeciesTaskFailureParametersImpl
        {
            std::string species;
            unsigned int task;
        };
    }  // namespace detail

    //! \brief Failure that represents that a specific species cannot be assigned to a specific task
    struct SpeciesTaskFailure : public FailureReason
    {
        std::string species;
        unsigned int task;

       protected:
        //! Constructor
        explicit SpeciesTaskFailure(const detail::SpeciesTaskFailureParametersImpl& parameters);

        //! Constructor
        SpeciesTaskFailure(detail::SpeciesTaskFailureParametersImpl&& parameters);

        //! Constructor
        SpeciesTaskFailure(const std::string& species, unsigned int task);

        //! Constructor
        SpeciesTaskFailure(std::string&& species, unsigned int task);
    };
}  // namespace grstapse