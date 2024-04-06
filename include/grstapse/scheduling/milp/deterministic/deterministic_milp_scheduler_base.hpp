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
//  Global
#include <unordered_map>
// Local
#include "grstapse/common/utilities/hash_extension.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_tasks_info.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_transitions_info.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"
// endregion

namespace grstapse
{
    // Forward Declarations
    class ConfigurationBase;
    class Robot;

    /*!
     * \brief Abstract base class for MILP formulations to solve deterministic robot scheduling problems
     *
     * \see DeterministicMilpScheduler
     * \see ScenarioMilpSubscheduler
     */
    class DeterministicMilpSchedulerBase : public MilpSchedulerBase
    {
       public:
        // region Special Member Functions
        DeterministicMilpSchedulerBase()                                          = delete;
        DeterministicMilpSchedulerBase(const DeterministicMilpSchedulerBase&)     = delete;
        DeterministicMilpSchedulerBase(DeterministicMilpSchedulerBase&&) noexcept = default;
        virtual ~DeterministicMilpSchedulerBase()                                 = default;
        DeterministicMilpSchedulerBase& operator=(const DeterministicMilpSchedulerBase&) = delete;
        DeterministicMilpSchedulerBase& operator=(DeterministicMilpSchedulerBase&&) noexcept = default;
        // endregion
       protected:
        /*!
         * Constructor
         *
         * \param problem_inputs
         * \param mutex_indicators
         * \param name_scheme
         * \param reduced_mutex_constraints
         */
        DeterministicMilpSchedulerBase(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<MutexIndicators>& mutex_indicators,
            const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
            const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface);

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> setupData() final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model) final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskTransitionVariables(GRBModel& model) final override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model) final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTaskConstraints(GRBModel& model) final override;

        //! \copydoc MilpSchedulerBase
        std::shared_ptr<const FailureReason> createTransitionConstraints(GRBModel& model) final override;

        //! \copydoc MilpSolverBase
        std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model) final override;

        DmsAllTasksInfo m_task_info;
        DmsAllTransitionsInfo m_transition_info;
        GRBVar m_makespan;

        std::shared_ptr<const DmsNameSchemeBase> m_name_scheme;

        friend class MonolithicStochasticMilpScheduler;
        friend class BendersStochasticMilpScheduler;
        friend class HeuristicApproximationStochasticScheduler;
    };

}  // namespace grstapse