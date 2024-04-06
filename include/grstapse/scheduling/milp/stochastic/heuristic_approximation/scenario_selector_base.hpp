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
#include <optional>
#include <tuple>
#include <vector>
// External
// Local
// endregion

namespace grstapse
{
    // region Forward Declarations
    class Timer;
    class MaskedCompleteSampledEuclideanGraphMotionPlanner;
    class SchedulerProblemInputs;
    class Task;
    // endregion

    /*!
     * \class ScenarioSelectorBase
     * \brief
     */
    class ScenarioSelectorBase
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        ScenarioSelectorBase() = delete;
        //! Copy Constructor
        ScenarioSelectorBase(const ScenarioSelectorBase&) = default;
        //! Move Constructor
        ScenarioSelectorBase(ScenarioSelectorBase&&) noexcept = default;
        //! Destructor
        ~ScenarioSelectorBase() = default;
        //! Copy Assignment Operator
        ScenarioSelectorBase& operator=(const ScenarioSelectorBase&) = default;
        //! Move Assignment Operator
        ScenarioSelectorBase& operator=(ScenarioSelectorBase&&) noexcept = default;
        // endregion

        explicit ScenarioSelectorBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

        [[nodiscard]] virtual std::optional<std::vector<bool>> createMask(
            Timer& timer,
            const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
            unsigned int num_samples,
            unsigned int beta,
            float gamma,
            float timeout) = 0;

       protected:
        /*!
         * \returns The two vertex indices connecting the terminal configuration of task \p i to the initial
         *           configuration of \p j
         */
        [[nodiscard]] static std::pair<unsigned int, unsigned int> getEdge(const std::shared_ptr<const Task>& i,
                                                                           const std::shared_ptr<const Task>& j);
        //! \returns The two vertex indices connecting the initial and terminal configuration of task \p i
        [[nodiscard]] static std::pair<unsigned int, unsigned int> getEdge(const std::shared_ptr<const Task>& task);
        /*!
         * \returns The two vertex indices connecting the terminal configuration of task \p i to the initial
         *           configuration of \p j
         */
        [[nodiscard]] std::pair<unsigned int, unsigned int> getEdge(unsigned int i, unsigned int j) const;
        //! \returns The two vertex indices connecting the initial and terminal configuration of task \p i
        [[nodiscard]] std::pair<unsigned int, unsigned int> getEdge(unsigned int i) const;

        std::shared_ptr<const SchedulerProblemInputs> m_problem_inputs;
    };  // class ScenarioSelectorBase
}  // namespace grstapse