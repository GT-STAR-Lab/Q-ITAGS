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
#include <memory>
#include <vector>
// External
#include <concurrencpp/concurrencpp.h>
// Local
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/heuristic_scenario_selector.hpp"
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_base.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    template <typename>
    class UndirectedGraphEdge;
    class EuclideanGraphConfiguration;
    class EuclideanGraphEnvironment;
    class MaskedCompleteSampledEuclideanGraphMotionPlanner;
    // endregion

    /*!
     * \class HeuristicApproximationStochasticScheduler
     * \brief
     */
    class HeuristicApproximationStochasticScheduler : public StochasticMilpSchedulerBase
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        HeuristicApproximationStochasticScheduler() = delete;
        //! Copy Constructor
        HeuristicApproximationStochasticScheduler(const HeuristicApproximationStochasticScheduler&) = delete;
        //! Move Constructor
        HeuristicApproximationStochasticScheduler(HeuristicApproximationStochasticScheduler&&) noexcept = default;
        //! Destructor
        ~HeuristicApproximationStochasticScheduler() = default;
        //! Copy Assignment Operator
        HeuristicApproximationStochasticScheduler& operator=(const HeuristicApproximationStochasticScheduler&) = delete;
        //! Move Assignment Operator
        HeuristicApproximationStochasticScheduler& operator=(HeuristicApproximationStochasticScheduler&&) noexcept =
            default;
        // endregion

        explicit HeuristicApproximationStochasticScheduler(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

        explicit HeuristicApproximationStochasticScheduler(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::function<std::shared_ptr<ScenarioSelectorBase>(
                const std::shared_ptr<const SchedulerProblemInputs>&)>& scenario_selector_constructor);

       protected:
        //! \copydoc StochasticMilpSchedulerBase
        [[nodiscard]] std::shared_ptr<const FailureReason> createMask(Timer& timer,
                                                                      float timeout,
                                                                      float gamma) final override;

        //! \copydoc StochasticMilpSchedulerBase
        [[nodiscard]] unsigned int numFScenarios() const final override;

        using MilpSolverBase::createModel;
        using MilpSolverBase::resolve;
        using MilpSolverBase::solveMilp;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model) override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model) override;

        std::shared_ptr<ScenarioSelectorBase> m_scenario_selector;
        unsigned int m_num_f_samples;
    };  // class HeuristicApproximationStochasticScheduler
}  // namespace grstapse