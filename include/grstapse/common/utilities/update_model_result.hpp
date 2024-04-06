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

namespace grstapse
{
    // Forward Declarations
    class FailureReason;

    //! Possible result types from attempting to update the model
    enum class UpdateModelResultType : uint8_t
    {
        e_no_update,
        e_updated,
        e_failure
    };

    //! Possible results from attempting to update the model
    class UpdateModelResult
    {
       public:
        //! Constructor
        explicit UpdateModelResult(UpdateModelResultType update_result_type);

        //! Constructor for failure
        explicit UpdateModelResult(const std::shared_ptr<const FailureReason>& failure_reason);

        //! \returns The type of update result
        [[nodiscard]] inline UpdateModelResultType type() const;

        //! \returns The reason for failure if there is one
        [[nodiscard]] inline const std::shared_ptr<const FailureReason>& failureReason() const;

       private:
        UpdateModelResultType m_type;
        std::shared_ptr<const FailureReason> m_failure_reason;
    };

    // Inline Functions
    UpdateModelResultType UpdateModelResult::type() const
    {
        return m_type;
    }

    const std::shared_ptr<const FailureReason>& UpdateModelResult::failureReason() const
    {
        return m_failure_reason;
    }

}  // namespace grstapse