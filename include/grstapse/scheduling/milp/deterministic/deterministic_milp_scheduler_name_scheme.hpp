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

// Local
#include "grstapse/scheduling/milp/deterministic/dms_name_scheme_base.hpp"

#include <stdexcept>

namespace grstapse
{
    /*!
     * Main name scheme for the standard DMScheduler
     */
    class DeterministicMilpSchedulerNameScheme : public DmsNameSchemeBase
    {
       public:
        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createTaskStartName(unsigned int task_nr) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createTransitionDurationVariableName(unsigned int first,
                                                                       unsigned int second) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createMutexIndicatorName(unsigned int first,
                                                           unsigned int second) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createMakespanVariableName() const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createTaskDurationConstraintName(unsigned int task_nr) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createTaskStartLowerBoundConstraintName(unsigned int task_nr) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createPrecedenceConstraintName(unsigned int first,
                                                                 unsigned int second) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createMutexConstraintName(unsigned int first,
                                                            unsigned int second) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createTransitionDurationLowerBoundConstraintName(
            unsigned int first,
            unsigned int second) const final override;

        //! \copydoc DmsNameSchemeBase
        [[nodiscard]] std::string createMakespanConstraintName(unsigned int task_nr) const final override;
    };

}  // namespace grstapse