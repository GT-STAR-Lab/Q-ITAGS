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

// Global
#include <tuple>
#include <unordered_map>
#include <vector>
// Local
#include "grstapse/common/utilities/custom_concepts.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_base.hpp"

namespace grstapse
{
    /*!
     * \brief Stub to allow solving the stochastic scheduling MILP with fixed mutex indicators and y indicators
     */
    class [[deprecated]] BendersStochasticLpSubscheduler : public StochasticMilpSchedulerBase
    {
       public:
        //! Constructor
        explicit BendersStochasticLpSubscheduler(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
                                                 const std::shared_ptr<MutexIndicators>& mutex_indicators,
                                                 const std::shared_ptr<std::vector<GRBVar>>& y_indicators,
                                                 const std::shared_ptr<const SmsNameSchemeBase>& name_scheme);

        /*!
         * \tparam ReturnType
         * \tparam Variable
         *
         * \param master_mutex_indicators
         * \param master_y_indicators
         *
         * \returns The optimality cut based on the dual
         */
        template <DualCutReturnType ReturnType, DualCutVariableType Variable>
        [[nodiscard]] ReturnType dualCut(
            std::unordered_map<std::pair<unsigned int, unsigned int>, Variable>& master_mutex_indicators,
            std::vector<Variable>& master_y_indicators)
        {
            ReturnType rv = 0.0;
            for(const std::unique_ptr<DeterministicMilpSubscheduler>& subscheduler: m_subschedulers)
            {
                rv += subscheduler->dualCut<ReturnType, Variable>(*m_model, master_mutex_indicators);
            }
            /*!
             * This variable intentionally does not follow the formatting guide
             *
             * \see https://en.wikipedia.org/wiki/Big_M_method
             */
            const double M = getM();
            for(unsigned int q = 0; q < m_num_scenarios; ++q)
            {
                const double zeta = constraintDualValue(m_y_constraints[q]);
                rv -= M * zeta * master_y_indicators[q];
            }

            return rv;
        }

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model) final override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model) final override;

        //! Add initial cuts to the master model
        std::shared_ptr<const FailureReason> createInitialCuts(GRBModel& model,
                                                               GRBVar& master_makespan,
                                                               std::vector<GRBVar>& master_y_indicators);

        //! \copydoc StochasticMilpSchedulerBase
        [[nodiscard]] std::shared_ptr<const FailureReason> createMask(Timer& timer,
                                                                      float timeout,
                                                                      float gamma) final override;

        //! \copydoc StochasticMilpSchedulerBase
        [[nodiscard]] unsigned int numFScenarios() const final override;

       private:
        std::vector<GRBConstr> m_y_constraints;

        friend class BendersStochasticMilpScheduler;
    };
}  // namespace grstapse