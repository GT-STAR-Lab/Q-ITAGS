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
#include "grstapse/common/utilities/update_model_result.hpp"

// Global
#include <cassert>

namespace grstapse
{
    UpdateModelResult::UpdateModelResult(UpdateModelResultType update_result_type)
        : m_type(update_result_type)
    {
        assert(update_result_type != UpdateModelResultType::e_failure);
    }

    UpdateModelResult::UpdateModelResult(const std::shared_ptr<const FailureReason>& failure_reason)
        : m_type(UpdateModelResultType::e_failure)
        , m_failure_reason(failure_reason)
    {}

}  // namespace grstapse