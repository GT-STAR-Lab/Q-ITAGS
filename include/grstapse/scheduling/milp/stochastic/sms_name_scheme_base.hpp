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
#include "grstapse/scheduling/milp/ms_name_scheme_base.hpp"

namespace grstapse
{
    /*!
     * \brief Abstract base class for naming MILP variables and constraints for stochastic scheduling
     */
    struct SmsNameSchemeBase : public MsNameSchemeBase
    {
        //! \returns The name for a y indicator for scenario \p q
        [[nodiscard]] virtual std::string createYIndicatorName(unsigned int q) const = 0;

        //! \returns The name for a y constraint for scenario \p q
        [[nodiscard]] virtual std::string createYConstraintName(unsigned int q) const = 0;
    };

}  // namespace grstapse