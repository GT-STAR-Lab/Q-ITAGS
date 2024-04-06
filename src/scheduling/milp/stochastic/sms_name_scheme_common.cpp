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
#include "grstapse/scheduling/milp/stochastic/sms_name_scheme_common.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    std::string SmsNameSchemeCommon::createYIndicatorName(unsigned int q) const
    {
        return fmt::format("yi^{0:d}", q);
    }
    std::string SmsNameSchemeCommon::createYConstraintName(unsigned int q) const
    {
        return fmt::format("yc^{0:d}", q);
    }
    std::string SmsNameSchemeCommon::createMakespanVariableName() const
    {
        return constants::k_makespan;
    }
}  // namespace grstapse