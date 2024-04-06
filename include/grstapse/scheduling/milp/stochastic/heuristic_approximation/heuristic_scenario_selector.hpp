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
#include <tuple>
#include <vector>
// External
// Local
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/scenario_selector_base.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    class SchedulerProblemInputs;
    class EuclideanGraphEnvironment;
    // endregion

    /*!
     * \class HeuristicScenarioSelector
     * \brief
     */
    class HeuristicScenarioSelector : public ScenarioSelectorBase
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        HeuristicScenarioSelector() = delete;
        //! Copy Constructor
        HeuristicScenarioSelector(const HeuristicScenarioSelector&) = default;
        //! Move Constructor
        HeuristicScenarioSelector(HeuristicScenarioSelector&&) noexcept = default;
        //! Destructor
        ~HeuristicScenarioSelector() = default;
        //! Copy Assignment Operator
        HeuristicScenarioSelector& operator=(const HeuristicScenarioSelector&) = default;
        //! Move Assignment Operator
        HeuristicScenarioSelector& operator=(HeuristicScenarioSelector&&) noexcept = default;
        // endregion

        explicit HeuristicScenarioSelector(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

        [[nodiscard]] std::optional<std::vector<bool>> createMask(
            Timer& timer,
            const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
            unsigned int num_samples,
            unsigned int beta,
            float gamma,
            float timeout) override;

       private:
        [[nodiscard]] float label(const std::shared_ptr<EuclideanGraphEnvironment>& environment,
                                  std::vector<std::pair<unsigned int, unsigned int>> task_edges,
                                  std::vector<std::pair<unsigned int, unsigned int>> precedence_transition_edges,
                                  std::vector<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>>
                                      mutex_transition_edges) const;

    };  // class HeuristicScenarioSelector
}  // namespace grstapse