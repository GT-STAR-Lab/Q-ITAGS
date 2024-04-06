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
#include <memory>
// External
#include "gurobi_c++.h"
// Local
#include "grstapse/scheduling/milp/deterministic/dms_task_info.hpp"

namespace grstapse
{
    // Forward Declarations
    class SchedulerProblemInputs;
    class SchedulerMotionPlannerInterfaceBase;
    class DmsNameSchemeBase;
    class FailureReason;
    class MutexIndicators;

    /*!
     * Contains info about all tasks needed for DeterministicMilpSchedulerBase
     *
     * \see DeterministicMilpSchedulerBase
     */
    class DmsAllTasksInfo
    {
       public:
        /*!
         * Constructor
         *
         * \param problem_inputs The inputs to the scheduling problem
         * \param name_scheme The scheme for naming variables and constraints
         * \param scheduler_motion_planner_interface An interface between the scheduler and motion planner
         */
        explicit DmsAllTasksInfo(
            const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
            const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& scheduler_motion_planner_interface);

        /*!
         * Sets up the data needed to create variables/constraints
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> setupData();

        /*!
         * Adds task variables to \p model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model);

        /*!
         * Adds constraints on the lowerbound for the start times of tasks to the model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createTaskLowerBoundConstraints(GRBModel& model);

        /*!
         * Tries to update the lower bound of task \p task_nr's timepoints
         *
         * \param task_nr The index of the task
         * \param robot The robot to compute a motion plan for
         *
         * \returns Whether the model was updated or not (or a failure occurred)
         */
        UpdateModelResult updateTaskLowerBound(unsigned int task_nr, const std::shared_ptr<const Robot>& robot);

        //! \returns The MILP variable representing the start of task \p task_nr
        [[nodiscard]] inline GRBVar& taskStartTimePointVariable(unsigned int task_nr);

        //! \returns A list of task indices in the order with which they start in the schedule
        [[nodiscard]] std::vector<unsigned int> scheduledOrder() const;

        //! \returns A list of timepoints for the start and finish of each of the tasks
        std::vector<std::pair<float, float>> timePoints() const;

        //! \returns The part of the optimality cut related to the tasks
        [[nodiscard]] double dualCut() const;

        //! \returns The duration of a specific task
        [[nodiscard]] inline float taskDuration(unsigned int task_nr) const;

        //! \returns The lower bound of a specific task
        [[nodiscard]] inline float taskLowerBound(unsigned int task_nr) const;

       private:
        std::shared_ptr<const SchedulerProblemInputs> m_problem_inputs;
        std::shared_ptr<const SchedulerMotionPlannerInterfaceBase> m_scheduler_motion_planner_interface;
        std::shared_ptr<const DmsNameSchemeBase> m_name_scheme;
        std::vector<DmsTaskInfo> m_task_infos;
    };

    // Inline Functions
    GRBVar& DmsAllTasksInfo::taskStartTimePointVariable(unsigned int task_nr)
    {
        return m_task_infos[task_nr].startTimePoint();
    }

    float DmsAllTasksInfo::taskDuration(unsigned int task_nr) const
    {
        return m_task_infos[task_nr].duration();
    }

    float DmsAllTasksInfo::taskLowerBound(unsigned int task_nr) const
    {
        return m_task_infos[task_nr].duration();
    }
}  // namespace grstapse