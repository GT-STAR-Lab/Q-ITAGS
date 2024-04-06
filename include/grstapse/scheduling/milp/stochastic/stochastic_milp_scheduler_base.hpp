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

// region Includes
// Global
#include <coroutine>
// External
#include <cppcoro/generator.hpp>
// Local
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_subscheduler.hpp"
#include "grstapse/scheduling/milp/stochastic/sms_name_scheme_common.hpp"
// endregion

namespace grstapse
{
    // region Forward Declaration
    class MaskedCompleteSampledEuclideanGraphMotionPlanner;
    // endregion

    /*!
     * \brief Base for using a MILP formulation to solve a stochastic robot scheduling problem
     *
     * \note We use gurobi as our MILP solver
     */
    class StochasticMilpSchedulerBase : public MilpSchedulerBase
    {
       public:
        //! Constructor
        explicit StochasticMilpSchedulerBase(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<MutexIndicators>& mutex_indicators,
            const std::shared_ptr<std::vector<GRBVar>>& y_indicators,
            const std::shared_ptr<const SmsNameSchemeBase>& name_scheme = std::make_shared<SmsNameSchemeCommon>(),
            bool bender_decomposition                                   = false);

        explicit StochasticMilpSchedulerBase(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<std::vector<GRBVar>>& y_indicators,
            const std::shared_ptr<const SmsNameSchemeBase>& name_scheme = std::make_shared<SmsNameSchemeCommon>(),
            bool bender_decomposition                                   = false);

        explicit StochasticMilpSchedulerBase(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<const SmsNameSchemeBase>& name_scheme = std::make_shared<SmsNameSchemeCommon>(),
            bool bender_decomposition                                   = false);

       protected:
        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> setupData() final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model) final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskTransitionVariables(GRBModel& model) override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model) override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjective(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskConstraints(GRBModel& model) final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTransitionConstraints(GRBModel& model) final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const ScheduleBase> createSchedule(GRBModel& model) final override;

        //! \copydoc SchedulerBase
        [[nodiscard]] std::shared_ptr<const SchedulerResult> computeSchedule() override;

        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createMask(Timer& timer,
                                                                              float timeout,
                                                                              float gamma) = 0;

        [[nodiscard]] virtual unsigned int numFScenarios() const = 0;

        [[nodiscard]] cppcoro::generator<float> sprtSample(unsigned int num_g);

        [[nodiscard]] float singleSample(unsigned int index);

        GRBVar m_makespan;
        unsigned int m_num_scenarios;                         //!< This must be before m_y_indicators
        std::shared_ptr<std::vector<GRBVar>> m_y_indicators;  //!< This must be after m_num_scenarios
        std::vector<std::unique_ptr<DeterministicMilpSubscheduler>> m_subschedulers;
        float m_alpha_q;  //!< alpha * q
        std::shared_ptr<const SmsNameSchemeBase> m_name_scheme;
        std::vector<std::pair<unsigned int, unsigned int>> m_precedence_set_mutex_constraints;
        std::vector<float> m_prior_sprt;
        std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner> m_motion_planner;
    };

}  // namespace grstapse