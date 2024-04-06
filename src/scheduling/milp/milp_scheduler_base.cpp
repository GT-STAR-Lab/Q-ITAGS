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
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"

// Local
#include "grstapse/common/milp/milp_failure_reason.hpp"
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"

namespace grstapse
{
    unsigned int MilpSchedulerBase::s_num_iterations = 0;

    MilpSchedulerBase::MilpSchedulerBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
                                         const std::shared_ptr<MutexIndicators>& mutex_indicators,
                                         bool benders_decomposition)
        : SchedulerBase(problem_inputs)
        , MilpSolverBase(benders_decomposition)
        , m_mutex_indicators(mutex_indicators)
    {}

    unsigned int MilpSchedulerBase::numIterations()
    {
        return s_num_iterations;
    }

    std::shared_ptr<const grstapse::SchedulerResult> MilpSchedulerBase::computeSchedule()
    {
        std::shared_ptr<MilpSolverResult> result = solveMilp(m_problem_inputs->schedulerParameters());
        if(result->failure())
        {
            return std::make_shared<SchedulerResult>(result->failureReason());
        }

        s_num_iterations += result->numIterations();

        if(auto schedule = createSchedule(*result->model()); schedule)
        {
            return std::make_shared<SchedulerResult>(schedule);
        }
        return std::make_shared<SchedulerResult>(std::make_shared<MilpFailureReason>());
    }

    std::shared_ptr<const FailureReason> MilpSchedulerBase::createVariables(GRBModel& model)
    {
        m_mutex_indicators->createVariables(model);

        if(std::shared_ptr<const FailureReason> failure_reason = createTaskVariables(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createTaskTransitionVariables(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createObjectiveVariables(model); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> MilpSchedulerBase::createConstraints(GRBModel& model)
    {
        if(std::shared_ptr<const FailureReason> failure_reason = createTaskConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createTransitionConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createObjectiveConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    double MilpSchedulerBase::getM() const
    {
        return m_problem_inputs->scheduleWorstMakespan();
    }
}  // namespace grstapse