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
        /*!
         * \brief Hack for designated initializers
         */
        struct SpeciesTaskPairFailureParametersImpl
        {
            std::string species;
            unsigned int predecessor_task_index;
            unsigned int successor_task_index;
        };
    }  // namespace detail

    /*!
     * \brief Failure that represents that a specific species cannot be assigned to both task's i and j
     */
    struct SpeciesTaskPairFailure : public FailureReason
    {
        std::string species;
        unsigned int predecessor_task_index;
        unsigned int successor_task_index;

       protected:
        //! Constructor
        SpeciesTaskPairFailure(const detail::SpeciesTaskPairFailureParametersImpl& parameters);

        //! Constructor
        SpeciesTaskPairFailure(detail::SpeciesTaskPairFailureParametersImpl&& parameters);

        //! Constructor
        SpeciesTaskPairFailure(const std::string& species,
                               unsigned int predecessor_task_index,
                               unsigned int successor_task_index);

        //! Constructor
        SpeciesTaskPairFailure(std::string&& species,
                               unsigned int predecessor_task_index,
                               unsigned int successor_task_index);
    };
}  // namespace grstapse