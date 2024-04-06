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

// region Includes
// Local
#include "grstapse/scheduling/schedule_base.hpp"
// endregion

namespace grstapse
{
    /*!
     * \class StochasticSchedule
     * \brief Container for a stochastic schedule for a set of tasks with constraints
     */
    class StochasticSchedule : public ScheduleBase
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        StochasticSchedule() = default;
        //! Copy Constructor
        StochasticSchedule(const StochasticSchedule&) = delete;
        //! Move Constructor
        StochasticSchedule(StochasticSchedule&&) noexcept = default;
        //! Destructor
        ~StochasticSchedule() = default;
        //! Copy Assignment Operator
        StochasticSchedule& operator=(const StochasticSchedule&) = delete;
        //! Move Assignment Operator
        StochasticSchedule& operator=(StochasticSchedule&&) noexcept = default;
        // endregion

        /*!
         * \brief Full Constructor
         *
         * \param makespan The total execution time for the schedule
         * \param precedence_set_mutex_constraints A list of the precedence set mutex constraints
         */
        StochasticSchedule(const float makespan,
                           const std::vector<std::pair<unsigned int, unsigned int>>& precedence_set_mutex_constraints);

        //! \copydoc ScheduleBase
        nlohmann::json serializeToJson(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs) const final override;
    };  // class StochasticSchedule
}  // namespace grstapse