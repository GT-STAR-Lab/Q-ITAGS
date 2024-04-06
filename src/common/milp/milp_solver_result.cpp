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
#include "grstapse/common/milp/milp_solver_result.hpp"

// Global
#include <cassert>

namespace grstapse
{
    MilpSolverResult::MilpSolverResult(const std::shared_ptr<GRBModel>& model)
        : m_model(model)
        , m_num_iterations(0)
        , m_failure_reason(nullptr)
    {
        assert(model);
    }

    MilpSolverResult::MilpSolverResult(const std::shared_ptr<const FailureReason>& failure_reason,
                                       unsigned int num_iterations)
        : m_model(nullptr)
        , m_num_iterations(num_iterations)
        , m_failure_reason(failure_reason)
    {
        assert(failure_reason);
    }
}  // namespace grstapse