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
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_subscheduler.hpp"
#include "grstapse/scheduling/milp/stochastic/benders/benders_stochastic_milp_scheduler_base.hpp"

namespace grstapse
{
    /*!
     * \brief Solves a stochastic robot scheduling problem through bender's decomposition
     *
     * This scheduler contains a master ILP problem that determines mutex reduction and y's and multiple LP
     * subproblems resenting all Q scenarios
     */
    class [[deprecated]] BendersParallelStochasticMilpScheduler : public BendersStochasticMilpSchedulerBase
    {
       public:
        //! Constructor
        explicit BendersParallelStochasticMilpScheduler(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<const SmsNameSchemeBase>& name_scheme = std::make_shared<SmsNameSchemeCommon>());

       protected:
        //! \copydoc MilpSolverBase
        void makeCuts(BendersCallback& callback) final override;
        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> setupData() final override;

        //! \copydoc BendersStochasticMilpSchedulerBase
        std::shared_ptr<const FailureReason> createInitialCuts(GRBModel& model) final override;

       private:
        std::unordered_map<std::pair<unsigned int, unsigned int>, bool> m_mutex_indicator_values;
        std::vector<bool> m_y_indicator_values;
        std::vector<float> m_subproblem_makespans;
        std::vector<std::shared_ptr<MutexIndicators>> m_subproblem_mutex_indicators;
        std::vector<std::unique_ptr<DeterministicMilpSubscheduler>> m_subschedulers;
    };

}  // namespace grstapse