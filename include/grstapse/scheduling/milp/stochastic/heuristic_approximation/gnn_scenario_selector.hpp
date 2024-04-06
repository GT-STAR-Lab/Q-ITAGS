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
// External
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
// Local
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/scenario_selector_base.hpp"
// endregion

namespace grstapse
{
    /*!
     * \class GnnScenarioSelector
     * \brief
     */
    class GnnScenarioSelector : public ScenarioSelectorBase
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        GnnScenarioSelector() = delete;
        //! Copy Constructor
        GnnScenarioSelector(const GnnScenarioSelector&) = default;
        //! Move Constructor
        GnnScenarioSelector(GnnScenarioSelector&&) noexcept = default;
        //! Destructor
        ~GnnScenarioSelector() = default;
        //! Copy Assignment Operator
        GnnScenarioSelector& operator=(const GnnScenarioSelector&) = default;
        //! Move Assignment Operator
        GnnScenarioSelector& operator=(GnnScenarioSelector&&) noexcept = default;
        // endregion

        explicit GnnScenarioSelector(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
                                     bool use_cpu        = true,
                                     unsigned int gpu_id = 0);

        [[nodiscard]] std::optional<std::vector<bool>> createMask(
            Timer& timer,
            const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
            unsigned int num_samples,
            unsigned int beta,
            float gamma,
            float timeout);

       private:
        void loadModel(const std::string& model_filepath, const std::string& model_parameters_filepath);

        [[nodiscard]] pybind11::dict buildDataDict(
            const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
            unsigned int num_samples);

        [[nodiscard]] pybind11::object buildGraph(
            const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
            unsigned int num_samples);

        bool m_use_cpu;
        unsigned int m_gpu_id;

        pybind11::scoped_interpreter m_scoped_interpreter;
        pybind11::module m_module;
        pybind11::object m_model;
        pybind11::object m_device;
    };  // class GnnScenarioSelector
}  // namespace grstapse