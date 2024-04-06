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

// Local
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"
#include "grstapse/scheduling/milp/stochastic/sms_name_scheme_common.hpp"

namespace grstapse
{
    /*!
     * \brief Base class for using bender's decomposition for stochastic robot scheduling
     *
     * This class handles much of setting up the master problem
     *
     *
     */
    class [[deprecated]] BendersStochasticMilpSchedulerBase : public MilpSchedulerBase
    {
       public:
        //! Constructor
        explicit BendersStochasticMilpSchedulerBase(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<const SmsNameSchemeBase>& name_scheme = std::make_shared<SmsNameSchemeCommon>());

       protected:
        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model) override;
        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskTransitionVariables(GRBModel& model) override;
        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model) override;
        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjective(GRBModel& model) override;
        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskConstraints(GRBModel& model) override;
        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTransitionConstraints(GRBModel& model) override;
        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model) override;

        /*!
         * \brief Add some initial cuts to the objective
         *
         * \param model The model to add the cuts to
         *
         * \returns Whether the initial cuts were successfully added
         */
        virtual std::shared_ptr<const FailureReason> createInitialCuts(GRBModel& model) = 0;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const ScheduleBase> createSchedule(GRBModel& model) override;

        unsigned int m_num_scenarios;
        float m_alpha_q;
        GRBVar m_alpha_robust_makespan;
        std::vector<GRBVar> m_task_stubs;
        std::vector<GRBVar> m_master_y_indicators;
        std::shared_ptr<const SmsNameSchemeBase> m_name_scheme;
    };

}  // namespace grstapse