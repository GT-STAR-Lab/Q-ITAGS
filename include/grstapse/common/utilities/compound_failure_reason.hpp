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
#include <memory>
#include <vector>
// Local
#include "grstapse/common/utilities/failure_reason.hpp"

namespace grstapse
{
    //! A list of failure reasons
    class CompoundFailureReason : public FailureReason
    {
       public:
        //! Constructor
        explicit CompoundFailureReason(const std::vector<std::shared_ptr<const FailureReason>>& reasons)
            : m_reasons(reasons)
        {}

        //! \returns A list of the underlying failure reasons
        [[nodiscard]] inline const std::vector<std::shared_ptr<const FailureReason>>& reasons() const;

       private:
        std::vector<std::shared_ptr<const FailureReason>> m_reasons;
    };

    // Inline Functions
    const std::vector<std::shared_ptr<const FailureReason>>& CompoundFailureReason::reasons() const
    {
        return m_reasons;
    }

}  // namespace grstapse