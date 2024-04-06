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
#include <unordered_map>
// External
#include <gurobi_c++.h>
// Local
#include "grstapse/common/utilities/custom_views.hpp"
#include "grstapse/common/utilities/update_model_result.hpp"
#include "grstapse/scheduling/milp/deterministic/transition_computation_status.hpp"

namespace grstapse
{
    // Forward Declarations
    class Robot;
    class Task;
    class DmsNameSchemeBase;
    class SchedulerMotionPlannerInterfaceBase;
    class FailureReason;

    /*!
     * Creates all the information for building MILP model components representing this task
     */
    class DmsTaskInfo
    {
       public:
        /*!
         * Constructor
         *
         * \param coalition The coalition of robots assigned to this task
         * \param plan_task_nr The index for this task in the plan
         * \param task The task
         * \param name_scheme The scheme for naming variables/constraints
         * \param motion_planner_interface The interface to the motion planner
         */
        DmsTaskInfo(CoalitionView coalition,
                    unsigned int plan_task_nr,
                    const std::shared_ptr<const Task>& task,
                    const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
                    const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface);

        /*!
         * Sets up the data for calculating constraints and bounds
         *
         * \returns The reason for failure if there is one
         */
        [[nodiscard]] std::shared_ptr<const FailureReason> setupData();

        //! Creates the start and finish timepoint variables and adds them to the model
        void createTimePointVariables(GRBModel& model);

        //! Creates a redundant lowerbound constraint in order to get the dual value
        void createLowerBoundConstraint(GRBModel& model);

        /*!
         * Tries to update the lower bound of this task's timepoints
         *
         * \param robot The robot to compute a motion plan for
         * \returns Whether the model was updated or not (or a failure occurred)
         */
        [[nodiscard]] UpdateModelResult updateLowerBound(const std::shared_ptr<const Robot>& robot);

        //! \returns The duration of the task
        [[nodiscard]] inline float duration() const;

        /*!
         * \returns The lowerbound for the start of the task
         *
         * \note Not all motion plans may have been calculated, so this could be based on heuristics
         */
        [[nodiscard]] inline float lowerBound() const;

        //! \returns The start timepoint
        [[nodiscard]] inline GRBVar& startTimePoint();

        //! \returns The start timepoint
        [[maybe_unused]] [[nodiscard]] inline const GRBVar& startTimePoint() const;

        //! \returns The list of robots assigned to this task
        [[nodiscard]] std::vector<std::shared_ptr<const Robot>> coalition() const;

        /*!
         * \f[
         *      \sum_{i \in I}  x_i^q \epsilon_i^q
         * \f]
         *
         * \returns The component of the optimality cut for this task
         */
        [[nodiscard]] double dualCut() const;

       protected:
        float m_duration;
        float m_lower_bound;
        unsigned int m_plan_task_nr;
        std::shared_ptr<const Task> m_task;
        std::unordered_map<std::shared_ptr<const Robot>, std::pair<TransitionComputationStatus, float>> m_coalition;
        GRBVar m_start_time_point;
        GRBConstr m_lower_bound_constraint;

        std::shared_ptr<const DmsNameSchemeBase> m_name_scheme;
        std::shared_ptr<const SchedulerMotionPlannerInterfaceBase> m_motion_planner_interface;
    };

    // Inline Functions
    float DmsTaskInfo::duration() const
    {
        return m_duration;
    }

    float DmsTaskInfo::lowerBound() const
    {
        return m_lower_bound;
    }

    GRBVar& DmsTaskInfo::startTimePoint()
    {
        return m_start_time_point;
    }

    [[maybe_unused]] const GRBVar& DmsTaskInfo::startTimePoint() const
    {
        return m_start_time_point;
    }

}  // namespace grstapse