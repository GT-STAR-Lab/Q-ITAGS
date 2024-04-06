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
#include "grstapse/scheduling/milp/ms_name_scheme_base.hpp"

namespace grstapse
{
    /*!
     * \brief Abstract base class for naming MILP variables and constraints for deterministic scheduling
     */
    class DmsNameSchemeBase : public MsNameSchemeBase
    {
       public:
        //! \returns The name for the task start MILP variable for \p task_nr
        [[nodiscard]] virtual std::string createTaskStartName(unsigned int task_nr) const = 0;

        //! \returns The name of the transition duration variable for \p i and \p j
        [[nodiscard]] virtual std::string createTransitionDurationVariableName(unsigned int i,
                                                                               unsigned int j) const = 0;

        //! \returns The name for the task duration constraint for \p task_nr
        [[nodiscard]] virtual std::string createTaskDurationConstraintName(unsigned int task_nr) const = 0;

        //! \returns The name of the task lowerbound constraint for \p task_nr
        [[nodiscard]] virtual std::string createTaskStartLowerBoundConstraintName(unsigned int task_nr) const = 0;

        //! \returns The name for the precedence constraint from \p i to \p j
        [[nodiscard]] virtual std::string createPrecedenceConstraintName(unsigned int i, unsigned int j) const = 0;

        //! \returns The name for the mutex constraint between \p i and \p j when it is resolve as \p i -> \p j
        [[nodiscard]] virtual std::string createMutexConstraintName(unsigned int first, unsigned int second) const = 0;

        //! \returns The name for the lowerbound constraint on the transition duration
        [[nodiscard]] virtual std::string createTransitionDurationLowerBoundConstraintName(
            unsigned int first,
            unsigned int second) const = 0;

        //! \returns The name for the makespan constraint
        [[nodiscard]] virtual std::string createMakespanConstraintName(unsigned int task_nr) const = 0;
    };

}  // namespace grstapse