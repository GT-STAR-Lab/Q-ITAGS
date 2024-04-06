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
#include "grstapse/task_allocation/itags/percent_over_schedule.hpp"

#include "grstapse/common/utilities/logger.hpp"

namespace grstapse
{
    PercentOverSchedule::PercentOverSchedule(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        const std::function<void(const std::shared_ptr<const SchedulerResult>&)>& on_failure,
        std::function<void(const std::shared_ptr<const SchedulerResult>&)> on_success)
        : m_problem_inputs(problem_inputs)
        , m_create_scheduler(
              [](const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
              {
                  return std::make_shared<DeterministicMilpScheduler>(problem_inputs);
              })
        , m_on_failure(on_failure)
        , m_on_success(on_success)
    {}

    PercentOverSchedule::PercentOverSchedule(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
        const std::function<std::shared_ptr<SchedulerBase>(const std::shared_ptr<const SchedulerProblemInputs>&)>&
            create_scheduler,
        const std::function<void(const std::shared_ptr<const SchedulerResult>&)>& on_failure,
        std::function<void(const std::shared_ptr<const SchedulerResult>&)> on_success)
        : m_problem_inputs(problem_inputs)
        , m_create_scheduler(create_scheduler)
        , m_on_failure(on_failure)
        , m_on_success(on_success)
    {}

    float PercentOverSchedule::operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const
    {
        return operator()(node.get());
    }

    float PercentOverSchedule::operator()(IncrementalTaskAllocationNode* node) const
    {
        float pos = (computeMakespan(node) - m_problem_inputs->scheduleMax()) /
                    (m_problem_inputs->scheduleWorstMakespan() - m_problem_inputs->scheduleMax());
        if(pos > 0.0)
        {
            return pos;
        }
        return 0.0;
    }

    float PercentOverSchedule::computeMakespan(IncrementalTaskAllocationNode* node) const
    {
        // Calculate the Makespan
        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(m_problem_inputs, node->allocation());
        auto scheduler                = m_create_scheduler(scheduler_problem_inputs);
        std::shared_ptr<const SchedulerResult> result = scheduler->solve();
        if(result->failed())
        {
            m_on_failure(result);
            node->setSchedule(nullptr);
            return std::numeric_limits<float>::infinity();
        }

        m_on_success(result);
        node->setSchedule(result->schedule());
        return node->schedule()->makespan();
    }
}  // namespace grstapse