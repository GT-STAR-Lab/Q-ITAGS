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
//  External
#include <concurrencpp/concurrencpp.h>
// Local
#include "grstapse/scheduling/milp/stochastic/stochastic_milp_scheduler_base.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    class CompleteSampledEuclideanGraphMotionPlanner;
    // endregion

    /*!
     * \brief Solves the robot scheduling problem with temporal uncertainty as a monolithic MILP
     */
    class MonolithicStochasticMilpScheduler : public StochasticMilpSchedulerBase
    {
       public:
        //! Constructor
        explicit MonolithicStochasticMilpScheduler(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

       protected:
        //! \copydoc MilpSolverBase
        [[nodiscard]] std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model) final override;

        //! \copydoc StochasticMilpSchedulerBase
        [[nodiscard]] std::shared_ptr<const FailureReason> createMask(Timer& timer,
                                                                      float timeout,
                                                                      float gamma) final override;

        //! \copydoc StochasticMilpSchedulerBase
        [[nodiscard]] unsigned int numFScenarios() const final override;
    };

}  // namespace grstapse