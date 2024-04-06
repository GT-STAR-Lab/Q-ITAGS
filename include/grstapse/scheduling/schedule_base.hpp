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
//  Global
#include <memory>
#include <string>
#include <tuple>
#include <vector>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    class SchedulerProblemInputs;
    // endregion

    /*!
     * \brief Base class for a container that represents information about a schedule
     */
    class ScheduleBase : public Noncopyable
    {
       public:
        //! Default Constructor
        ScheduleBase();

        //! virtual destructor to make this class polymorphic
        virtual ~ScheduleBase() = default;

        //! Constructor
        ScheduleBase(float makespan,
                     const std::vector<std::pair<unsigned int, unsigned int>>& precedence_set_mutex_constraints);

        //! \returns The makespan (or the total execution time) of the schedule
        [[nodiscard]] inline float makespan() const;

        //! \returns A list of the precedence set mutex constraints
        [[nodiscard]] inline const std::vector<std::pair<unsigned int, unsigned int>>& precedenceSetMutexConstraints()
            const;

        //! Writes the schedule to file
        virtual nlohmann::json serializeToJson(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs) const = 0;

       protected:
        float m_makespan;
        std::vector<std::pair<unsigned int, unsigned int>> m_precedence_set_mutex_constraints;
    };

    // Inline Functions
    float ScheduleBase::makespan() const
    {
        return m_makespan;
    }

    const std::vector<std::pair<unsigned int, unsigned int>>& ScheduleBase::precedenceSetMutexConstraints() const
    {
        return m_precedence_set_mutex_constraints;
    }
}  // namespace grstapse