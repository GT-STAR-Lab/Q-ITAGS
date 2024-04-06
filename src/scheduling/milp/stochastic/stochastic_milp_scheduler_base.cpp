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
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_base.hpp"

// Local
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/timeout_failure.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/masked_complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/sequential_probability_ratio_test.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_schedule.hpp"
#include "grstapse/scheduling/schedule_base.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"

namespace grstapse
{
    StochasticMilpSchedulerBase::StochasticMilpSchedulerBase(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<MutexIndicators>& mutex_indicators,
        const std::shared_ptr<std::vector<GRBVar>>& y_indicators,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme,
        bool bender_decomposition)
        : MilpSchedulerBase(problem_inputs, mutex_indicators, bender_decomposition)
        , m_num_scenarios(problem_inputs->schedulerParameters()->get<unsigned int>(constants::k_num_scenarios))
        , m_y_indicators(y_indicators)
        , m_alpha_q(m_num_scenarios * problem_inputs->schedulerParameters()->get<float>(constants::k_gamma))
        , m_name_scheme(name_scheme)
    {}

    StochasticMilpSchedulerBase::StochasticMilpSchedulerBase(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<std::vector<GRBVar>>& y_indicators,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme,
        bool bender_decomposition)
        : MilpSchedulerBase(problem_inputs,
                            std::make_shared<MutexIndicators>(problem_inputs, name_scheme),
                            bender_decomposition)
        , m_num_scenarios(problem_inputs->schedulerParameters()->get<unsigned int>(constants::k_num_scenarios))
        , m_y_indicators(y_indicators)
        , m_alpha_q(m_num_scenarios * problem_inputs->schedulerParameters()->get<float>(constants::k_gamma))
        , m_name_scheme(name_scheme)
    {}

    StochasticMilpSchedulerBase::StochasticMilpSchedulerBase(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<const SmsNameSchemeBase>& name_scheme,
        bool bender_decomposition)
        : MilpSchedulerBase(problem_inputs,
                            std::make_shared<MutexIndicators>(problem_inputs, name_scheme),
                            bender_decomposition)
        , m_num_scenarios(problem_inputs->schedulerParameters()->get<unsigned int>(constants::k_num_scenarios))
        , m_y_indicators(std::make_shared<std::vector<GRBVar>>(m_num_scenarios))
        , m_alpha_q(m_num_scenarios * problem_inputs->schedulerParameters()->get<float>(constants::k_gamma))
        , m_name_scheme(name_scheme)
    {}

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::setupData()
    {
        m_subschedulers.reserve(m_num_scenarios);
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            m_subschedulers.push_back(
                std::make_unique<DeterministicMilpSubscheduler>(i, m_problem_inputs, m_mutex_indicators));
        }

        for(const std::unique_ptr<DeterministicMilpSubscheduler>& subscheduler: m_subschedulers)
        {
            if(std::shared_ptr<const FailureReason> failure_reason = subscheduler->setupData(); failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::createTaskVariables(GRBModel& model)
    {
        for(const std::unique_ptr<DeterministicMilpSubscheduler>& subscheduler: m_subschedulers)
        {
            if(std::shared_ptr<const FailureReason> failure_reason = subscheduler->createTaskVariables(model);
               failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::createTaskTransitionVariables(GRBModel& model)
    {
        for(const std::unique_ptr<DeterministicMilpSubscheduler>& subscheduler: m_subschedulers)
        {
            if(std::shared_ptr<const FailureReason> failure_reason = subscheduler->createTaskTransitionVariables(model);
               failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::createObjectiveVariables(GRBModel& model)
    {
        m_makespan =
            model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, m_name_scheme->createMakespanVariableName());
        for(unsigned int i = 0; i < m_num_scenarios; ++i)
        {
            if(std::shared_ptr<const FailureReason> failure_reason =
                   m_subschedulers[i]->createObjectiveVariables(model);
               failure_reason)
            {
                return failure_reason;
            }
            m_y_indicators->at(i) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, m_name_scheme->createYIndicatorName(i));
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::createObjective(GRBModel& model)
    {
        // Set all optimization to minimize (is the default, but we explicitly set anyway)
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        model.setObjective(GRBLinExpr(m_makespan));
        return nullptr;
    }

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::createTaskConstraints(GRBModel& model)
    {
        for(const std::unique_ptr<DeterministicMilpSubscheduler>& subscheduler: m_subschedulers)
        {
            if(std::shared_ptr<const FailureReason> failure_reason = subscheduler->createTaskConstraints(model);
               failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> StochasticMilpSchedulerBase::createTransitionConstraints(GRBModel& model)
    {
        for(const std::unique_ptr<DeterministicMilpSubscheduler>& subscheduler: m_subschedulers)
        {
            if(std::shared_ptr<const FailureReason> failure_reason = subscheduler->createTransitionConstraints(model);
               failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const ScheduleBase> StochasticMilpSchedulerBase::createSchedule(GRBModel& model)
    {
        const double makespan              = m_makespan.get(GRB_DoubleAttr_X);
        m_precedence_set_mutex_constraints = m_mutex_indicators->precedenceSet();
        return std::make_shared<const StochasticSchedule>(makespan, m_precedence_set_mutex_constraints);
    }

    std::shared_ptr<const SchedulerResult> StochasticMilpSchedulerBase::computeSchedule()
    {
        Timer timer;
        timer.start();

        const float timeout = m_problem_inputs->schedulerParameters()->get<float>(constants::k_timeout);
        const float gamma   = m_problem_inputs->schedulerParameters()->get<float>(constants::k_gamma);
        m_motion_planner    = std::dynamic_pointer_cast<MaskedCompleteSampledEuclideanGraphMotionPlanner>(
            m_problem_inputs->motionPlanner(0));
        if(m_motion_planner == nullptr)
        {
            throw createLogicError("Motion planner needs to be a 'MaskedCompleteSampledEuclideanGraphMotionPlanner'");
        }

        // Creates a mask for HA, but does nothing for SAA
        if(std::shared_ptr<const FailureReason> reason = createMask(timer, timeout, gamma); reason != nullptr)
        {
            return std::make_shared<SchedulerResult>(reason);
        }

        auto result = MilpSchedulerBase::computeSchedule();
        if(result->failed())
        {
            Logger::warn("Main Scheduling MILP failed");
            return result;
        }

        // Ignore SPRT
        if(not m_problem_inputs->schedulerParameters()->get<bool>(constants::k_use_sprt))
        {
            Logger::info("Not using SPRT");
            return result;
        }
        Logger::info("Using SPRT");

        if(timer.get() > timeout)
        {
            Logger::warn("Scheduler timed out");
            return std::make_shared<SchedulerResult>(std::make_shared<TimeoutFailure>());
        }

        const bool delta_percentage = m_problem_inputs->schedulerParameters()->get<bool>(constants::k_delta_percentage);
        const float delta           = delta_percentage
                                          ? 1.0f + m_problem_inputs->schedulerParameters()->get<float>(constants::k_delta)
                                          : m_problem_inputs->schedulerParameters()->get<float>(constants::k_delta);
        const float indifference_tolerance =
            m_problem_inputs->schedulerParameters()->get<float>(constants::k_indifference_tolerance);

        float makespan                     = variableValue(m_makespan);
        m_precedence_set_mutex_constraints = m_mutex_indicators->precedenceSet();

        SequentialProbabilityRatioTest sprt(gamma - indifference_tolerance, gamma + indifference_tolerance);
        const unsigned int num_g_scenarios = m_motion_planner->totalNumber() - numFScenarios();

        // Creates a new mask to ignore the previously used samples
        {
            std::vector<bool> mask(m_motion_planner->totalNumber(), true);
            for(unsigned int i = 0, end = numFScenarios(); i < end; ++i)
            {
                mask[i] = false;
            }
            m_motion_planner->setMask(mask);
        }

        m_prior_sprt = std::vector<float>(num_g_scenarios, -1.0f);
        while(not sprt.run(makespan, num_g_scenarios, sprtSample(num_g_scenarios)))
        {
            if(timer.get() > timeout)
            {
                Logger::warn("Scheduler timed out");
                return std::make_shared<SchedulerResult>(std::make_shared<TimeoutFailure>());
            }
            Logger::info("Increasing robust makespan.");
            if(delta_percentage)
            {
                makespan *= delta;
            }
            else
            {
                makespan += delta;
            }
        }

        return std::make_shared<SchedulerResult>(
            std::make_shared<StochasticSchedule>(makespan, m_precedence_set_mutex_constraints));
    }

    cppcoro::generator<float> StochasticMilpSchedulerBase::sprtSample(unsigned int num_g)
    {
        for(unsigned int i = 0; i < num_g; ++i)
        {
            co_yield singleSample(i);
        }

        co_return;
    }

    float StochasticMilpSchedulerBase::singleSample(unsigned int index)
    {
        if(m_prior_sprt[index] > -1.0f)
        {
            // Logger::info("Using cached sprt sample");
            return m_prior_sprt[index];
        }
        auto subproblem_mutex_indicator = std::make_shared<MutexIndicators>(m_problem_inputs, m_name_scheme, false);
        DeterministicMilpSubscheduler subscheduler(index, m_problem_inputs, subproblem_mutex_indicator, true);
        if(std::shared_ptr<MilpSolverResult> result = subscheduler.createModel(m_problem_inputs->schedulerParameters());
           result->failure())
        {
            Logger::warn("Subscheduler {0:d} failed to create model", index);
            return std::numeric_limits<float>::infinity();
        }

        for(auto& [p, s]: m_precedence_set_mutex_constraints)
        {
            if(subproblem_mutex_indicator->contains({p, s}))
            {
                fixVariable(subproblem_mutex_indicator->get({p, s}), 1.0);
            }
            else if(subproblem_mutex_indicator->contains({s, p}))
            {
                fixVariable(subproblem_mutex_indicator->get({s, p}), 0.0);
            }
            else
            {
                throw createLogicError("Cannot find mutex constraint");
            }
        }

        std::shared_ptr<const MilpSolverResult> result = subscheduler.resolve();

        if(result->failure())
        {
            Logger::warn("Subscheduler {0:d} failed to optimize model", index);
            m_prior_sprt[index] = std::numeric_limits<float>::infinity();
            return m_prior_sprt[index];
        }

        m_prior_sprt[index] = variableValue(subscheduler.makespanVariable());
        return m_prior_sprt[index];
    }
}  // namespace grstapse