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
#include "grstapse/task_allocation/itags/time_extended_task_allocation_metric.hpp"

namespace grstapse
{
    TimeExtendedTaskAllocationMetric::TimeExtendedTaskAllocationMetric(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        float alpha,
        const std::shared_ptr<NormalizedAllocationQuality>& naq,
        const std::shared_ptr<PercentOverSchedule>& pos)
        : m_alpha(alpha)
        , m_pos{pos != nullptr ? pos : std::make_shared<const PercentOverSchedule>(problem_inputs)}
        , m_naq{naq != nullptr ? naq : std::make_shared<const NormalizedAllocationQuality>(problem_inputs)}
    {}

    TimeExtendedTaskAllocationMetric::TimeExtendedTaskAllocationMetric(
        const detail::TimeExtendedTaskAllocationMetricParameters& parameters)
        : m_alpha(parameters.alpha)
        , m_pos{parameters.pos != nullptr ? parameters.pos
                                          : std::make_shared<const PercentOverSchedule>(parameters.problem_inputs)}
        , m_naq{parameters.naq != nullptr
                    ? parameters.naq
                    : std::make_shared<const NormalizedAllocationQuality>(parameters.problem_inputs)}
    {}

    TimeExtendedTaskAllocationMetric::TimeExtendedTaskAllocationMetric(
        detail::TimeExtendedTaskAllocationMetricParameters&& parameters)
        : m_alpha(parameters.alpha)
        , m_pos{parameters.pos != nullptr ? std::move(parameters.pos)
                                          : std::make_shared<const PercentOverSchedule>(parameters.problem_inputs)}
        , m_naq{parameters.naq != nullptr
                    ? std::move(parameters.naq)
                    : std::make_shared<const NormalizedAllocationQuality>(parameters.problem_inputs)}
    {}

    float TimeExtendedTaskAllocationMetric::operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const
    {
        float pos = m_pos->operator()(node);
        float naq = m_naq->operator()(node);
        return m_alpha * m_pos->operator()(node) + (1.0f - m_alpha) * m_naq->operator()(node);
    }
}  // namespace grstapse