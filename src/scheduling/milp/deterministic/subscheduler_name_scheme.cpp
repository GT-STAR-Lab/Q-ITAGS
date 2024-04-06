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
#include "grstapse/scheduling/milp/deterministic/subscheduler_name_scheme.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    SubschedulerNameScheme::SubschedulerNameScheme(unsigned int index)
        : m_index(index)
    {}

    std::string SubschedulerNameScheme::createTaskStartName(unsigned int task_nr) const
    {
        return fmt::format("ts_{0:d}^{1:d}", task_nr, m_index);
    }

    std::string SubschedulerNameScheme::createTransitionDurationVariableName(unsigned int i, unsigned int j) const
    {
        return fmt::format("trd_({0:d},{1:d})^{2:d}", i, j, m_index);
    }

    std::string SubschedulerNameScheme::createMutexIndicatorName(unsigned int first, unsigned int second) const
    {
        return fmt::format("mi_({0:d},{1:d})", first, second);
    }

    std::string SubschedulerNameScheme::createMakespanVariableName() const
    {
        return fmt::format("{0:s}^{1:d}", constants::k_makespan, m_index);
    }

    std::string SubschedulerNameScheme::createTaskDurationConstraintName(unsigned int task_nr) const
    {
        return fmt::format("tdc_{0:d}^{1:d}", task_nr, m_index);
    }

    std::string SubschedulerNameScheme::createTaskStartLowerBoundConstraintName(unsigned int task_nr) const
    {
        return fmt::format("tlbc_{0:d}^{1:d}", task_nr, m_index);
    }

    std::string SubschedulerNameScheme::createPrecedenceConstraintName(unsigned int i, unsigned int j) const
    {
        return fmt::format("pc_({0:d},{1:d})^{2:d}", i, j, m_index);
    }

    std::string SubschedulerNameScheme::createMutexConstraintName(unsigned int first, unsigned int second) const
    {
        return fmt::format("mc_({0:d},{1:d})^{2:d}", first, second, m_index);
    }

    std::string SubschedulerNameScheme::createTransitionDurationLowerBoundConstraintName(unsigned int first,
                                                                                         unsigned int second) const
    {
        return fmt::format("tdlbc_({0:d},{1:d})^{2:d}", first, second, m_index);
    }

    std::string SubschedulerNameScheme::createMakespanConstraintName(unsigned int task_nr) const
    {
        return fmt::format("mkc_{0:d}^{1:d}", task_nr, m_index);
    }

}  // namespace grstapse