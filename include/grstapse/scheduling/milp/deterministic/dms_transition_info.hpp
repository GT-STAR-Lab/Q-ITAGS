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
#include <tuple>
#include <vector>
// External
#include <gurobi_c++.h>
#include <robin_hood/robin_hood.hpp>
// Local
#include <fmt/format.h>

#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/common/utilities/custom_views.hpp"
#include "grstapse/common/utilities/update_model_result.hpp"
#include "grstapse/scheduling/milp/deterministic/transition_computation_status.hpp"
namespace grstapse
{
    // Forward Declarations
    class Robot;
    class ConfigurationBase;
    class DmsNameSchemeBase;
    class SchedulerMotionPlannerInterfaceBase;
    class FailureReason;

    /*!
     * Contains information used to build variables and constraints for the transition from tasks i to j
     */
    class DmsTransitionInfo
    {
       public:
        /*!
         * Constructor
         *
         * \param coalition A list of robots assigned to both
         * \param predecessor_index The index of a task
         * \param successor_index The index of another task
         * \param initial_configuration The terminal configuration of task_i (which is the initial configuration for
         * this transition) \param terminal_configuration The initial configuration of task_j (which is the terminal
         * configuration for this transition) \param name_scheme The scheme for naming variables/constraints \param
         * motion_planner_interface The interface to the motion planner
         */
        DmsTransitionInfo(CoalitionView coalition,
                          unsigned int predecessor_index,
                          unsigned int successor_index,
                          const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                          const std::shared_ptr<const ConfigurationBase>& terminal_configuration,
                          const std::shared_ptr<const DmsNameSchemeBase>& name_scheme,
                          const std::shared_ptr<const SchedulerMotionPlannerInterfaceBase>& motion_planner_interface);

        /*!
         * Sets up the data for calculating constraints and bounds
         *
         * \returns The reason for failure if there is one
         */
        [[nodiscard]] std::shared_ptr<const FailureReason> setupData();

        //! Creates a constraint representing the precedence transition
        void createPrecedenceTransitionConstraint(GRBModel& model,
                                                  GRBVar& predecessor,
                                                  double predecessor_duration,
                                                  GRBVar& successor);

        //! Creates a constraint representing this half of a mutex transition
        void createMutexTransitionConstraint(GRBModel& model,
                                             GRBVar& predecessor,
                                             double predecessor_duration,
                                             GRBVar& successor,
                                             GRBLinExpr&& mutex_indicator_component);

        /*!
         * Tries to update the lower bound of this transition's duration
         *
         * \param robot The robot to compute a motion plan for
         * \returns Whether the model was updated or not (or a failure occurred)
         */
        [[nodiscard]] UpdateModelResult updateLowerBound(const std::shared_ptr<const Robot>& robot);

        /*!
         * \brief Computes the part of the optimality cut for a specific precedence or mutex constraint
         *
         * \f[
         *     (d_i^q + x_{ij}^q) \beta_{ij}^q
         *     (d_i^q + x_{ij}^q - A_{ij}^q) \gamma_{ij}^q
         *     (d_j^q + x_{ji}^q - B_{ij}^q) \delta_{ij}^q}
         * \f]
         *
         * \f[
         *     A_{ij} = M(1-p_{ij})
         *     B_{ij} = Mp_{ij}
         * \f]
         *
         * \tparam ReturnType double, float, or GRBLinExpr
         *
         * \param predecessor_duration The task duration of the preceeding task
         * \param mutex_indicator_component 0, A_ij, or B_ij
         *
         * \returns The component of the optimality cut for this transition
         */
        template <DualCutReturnType ReturnType>
        [[nodiscard]] ReturnType dualCut(double predecessor_duration,
                                         ReturnType&& mutex_indicator_component = 0.0) const
        {
            const double beta_gamma_delta = constraintDualValue(m_transition_constraint);
            return (predecessor_duration + m_duration_lowerbound - mutex_indicator_component) * beta_gamma_delta;
        }

        //! \returns The lower bound on this transition's duration
        [[nodiscard]] inline float durationLowerBound() const;

       private:
        float m_duration_lowerbound;
        unsigned int m_predecessor_index;
        unsigned int m_successor_index;
        robin_hood::unordered_map<std::shared_ptr<const Robot>, std::pair<TransitionComputationStatus, float>>
            m_coalition;
        std::shared_ptr<const ConfigurationBase> m_initial_configuration;
        std::shared_ptr<const ConfigurationBase> m_terminal_configuration;
        GRBConstr m_transition_constraint;
        std::string m_transition_name;

        std::shared_ptr<const DmsNameSchemeBase> m_name_scheme;
        std::shared_ptr<const SchedulerMotionPlannerInterfaceBase> m_motion_planner_interface;
    };

    // Inline Functions
    float DmsTransitionInfo::durationLowerBound() const
    {
        return m_duration_lowerbound;
    }

}  // namespace grstapse