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
#include "grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp"

namespace grstapse
{
    TimeExtendedTaskAllocationQuality::TimeExtendedTaskAllocationQuality(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        float alpha,
        const std::shared_ptr<AllocationPercentageRemaining>& apr,
        const std::shared_ptr<NormalizedScheduleQuality>& nsq)
        : m_alpha(alpha)
        , m_apr{apr != nullptr ? apr : std::make_shared<const AllocationPercentageRemaining>(problem_inputs)}
        , m_nsq{nsq != nullptr ? nsq : std::make_shared<const NormalizedScheduleQuality>(problem_inputs)}
    {}

    TimeExtendedTaskAllocationQuality::TimeExtendedTaskAllocationQuality(
        const detail::TimeExtendedTaskAllocationQualityParameters& parameters)
        : m_alpha(parameters.alpha)
        , m_apr{parameters.apr != nullptr
                    ? parameters.apr
                    : std::make_shared<const AllocationPercentageRemaining>(parameters.problem_inputs)}
        , m_nsq{parameters.nsq != nullptr
                    ? parameters.nsq
                    : std::make_shared<const NormalizedScheduleQuality>(parameters.problem_inputs)}
    {}

    TimeExtendedTaskAllocationQuality::TimeExtendedTaskAllocationQuality(
        detail::TimeExtendedTaskAllocationQualityParameters&& parameters)
        : m_alpha(parameters.alpha)
        , m_apr{parameters.apr != nullptr
                    ? std::move(parameters.apr)
                    : std::make_shared<const AllocationPercentageRemaining>(parameters.problem_inputs)}
        , m_nsq{parameters.nsq != nullptr
                    ? std::move(parameters.nsq)
                    : std::make_shared<const NormalizedScheduleQuality>(parameters.problem_inputs)}
    {}

    float TimeExtendedTaskAllocationQuality::operator()(
        const std::shared_ptr<IncrementalTaskAllocationNode>& node) const
    {
        return m_alpha * m_apr->operator()(node) + (1.0f - m_alpha) * m_nsq->operator()(node);
    }
}  // namespace grstapse