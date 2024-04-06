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
#include <list>
#include <memory>
#include <tuple>
#include <unordered_map>
// External
#include <gurobi_c++.h>
// Local
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_base.hpp"
// endregion

namespace grstapse
{
    /*!
     * A scheduling algorothm that uses a MILP formulation to solve a deterministic robot scheduling problem
     */
    class DeterministicMilpScheduler : public DeterministicMilpSchedulerBase
    {
       public:
        // region Special Member Functions
        DeterministicMilpScheduler()                                      = delete;
        DeterministicMilpScheduler(const DeterministicMilpScheduler&)     = delete;
        DeterministicMilpScheduler(DeterministicMilpScheduler&&) noexcept = default;
        ~DeterministicMilpScheduler()                                     = default;
        DeterministicMilpScheduler& operator=(const DeterministicMilpScheduler&) = delete;
        DeterministicMilpScheduler& operator=(DeterministicMilpScheduler&&) noexcept = default;
        // endregion

        /*!
         * \brief Constructor
         *
         * \param problem_inputs Inputs for a scheduling problem
         */
        explicit DeterministicMilpScheduler(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

       protected:
        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjective(GRBModel& model) final override;

        //! \copydoc MilpSolverBase
        UpdateModelResult updateModel(GRBModel& model) override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const ScheduleBase> createSchedule(GRBModel& model) final override;
    };
}  // namespace grstapse