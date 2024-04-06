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
#include <robin_hood/robin_hood.hpp>

#include "grstapse/common/milp/milp_solver_base.hpp"
#include "grstapse/common/utilities/hash_extension.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_tasks_info.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_transitions_info.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"
#include "grstapse/scheduling/scheduler_base.hpp"

namespace grstapse
{
    // Forward Declaration
    class MilpSchedulerParameters;
    class MutexIndicators;
    class ScheduleBase;

    /*!
     * Abstract base class for scheduling algorithms that use MILP formulations
     */
    class MilpSchedulerBase
        : public SchedulerBase
        , public MilpSolverBase
    {
       public:
        // region Special Member Function
        MilpSchedulerBase()                             = delete;
        MilpSchedulerBase(const MilpSchedulerBase&)     = delete;
        MilpSchedulerBase(MilpSchedulerBase&&) noexcept = default;
        virtual ~MilpSchedulerBase()                    = default;
        MilpSchedulerBase& operator=(const MilpSchedulerBase&) = delete;
        MilpSchedulerBase& operator=(MilpSchedulerBase&&) noexcept = default;
        // endregion

        //! \returns The number of times a MILP optimization was run
        [[nodiscard]] static unsigned int numIterations();

       protected:
        //! Constructor
        explicit MilpSchedulerBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
                                   const std::shared_ptr<MutexIndicators>& mutex_indicators,
                                   bool benders_decomposition = false);

        //! \copydoc SchedulerBase
        std::shared_ptr<const SchedulerResult> computeSchedule() override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createVariables(GRBModel& model) override;

        /*!
         * \brief Add variables that are needed for the tasks (i.e. start and finish timepoints)
         *
         * \param model The model to add the constraints to
         *
         * \returns Whether the variables were successfully added
         */
        virtual std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model) = 0;

        /*!
         * \brief Add variables that are needed for the task transitions (i.e. mutex indicators)
         *
         * \param model The model to add the constraints to
         *
         * \returns Whether the variables were successfully added
         */
        virtual std::shared_ptr<const FailureReason> createTaskTransitionVariables(GRBModel& model) = 0;

        /*!
         * \brief Add variables that are needed for the objective function (i.e. makespan)
         *
         * \param model The model to add the constraints to
         *
         * \returns Whether the variables were successfully added
         */
        virtual std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model) = 0;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createConstraints(GRBModel& model) override;

        /*!
         * \brief Adds constraints that affect the tasks, but not task transitions (i.e. duration constraints)
         *
         * \param model The model to add the constraints to
         *
         * \returns Whether the constraints were successfully added
         */
        virtual std::shared_ptr<const FailureReason> createTaskConstraints(GRBModel& model) = 0;

        /*!
         * \brief Adds constraints that affect task transitions
         *
         * \param model The model to add the constraints to
         *
         * \returns Whether the constraints were successfully added
         *
         * \note This also add TP precedence constraints even if there is not a robot transition associated with them
         */
        virtual std::shared_ptr<const FailureReason> createTransitionConstraints(GRBModel& model) = 0;

        /*!
         * \brief Add constraints that affect the objective function
         *
         * \param model The model to add the constraints to
         *
         * \returns Whether the constraints were successfully added
         */
        virtual std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model) = 0;

        //! Builds a schedule from the MILP variables in \p model
        virtual std::shared_ptr<const ScheduleBase> createSchedule(GRBModel& model) = 0;

        /*!
         * \returns Big M
         * \see https://en.wikipedia.org/wiki/Big_M_method
         */
        [[nodiscard]] double getM() const;

        static unsigned int s_num_iterations;
        //! These are the mutex constraint ids after the precedence constraints have been removed
        std::shared_ptr<MutexIndicators> m_mutex_indicators;
    };

}  // namespace grstapse