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
#include <map>
#include <memory>
#include <mutex>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/timer.hpp"
#include "grstapse/species.hpp"

namespace grstapse
{
    // region Forward Declarations
    class EnvironmentBase;
    class ParametersBase;
    class MotionPlannerQueryResultBase;
    class ConfigurationBase;
    enum class ConfigurationType : uint8_t;
    // endregion

    /*!
     * \brief Abstract base class for motion planning algorithms
     */
    class MotionPlannerBase : public Noncopyable
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param parameters
         * \param environment
         */
        MotionPlannerBase(const std::shared_ptr<const ParametersBase>& parameters,
                          const std::shared_ptr<EnvironmentBase>& environment);

        //! Initialize Factory
        static void init();

        //! \returns The parameters for this motion planner
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& parameters() const;

        //! \returns A pointer to the environment representation
        [[nodiscard]] inline const std::shared_ptr<EnvironmentBase>& environment() const;

        /*!
         * \brief Queries for a path from \p initial_configuration to \p goal_configuration
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The result of the motion planning query
         */
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> query(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration);

        /*!
         * \brief Queries for the duration to execute the path from \p initial_configuration to \p
         *        goal_configuration
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The status of the planner and the path generated as the solution if possible
         */
        [[nodiscard]] float durationQuery(const std::shared_ptr<const Species>& species,
                                          const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                          const std::shared_ptr<const ConfigurationBase>& goal_configuration);

        /*!
         * \brief Checks if a path from \p start_state to \p goal_configuration has been memoized
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns Whether a path from \p start_state to \p goal_state has been memoized
         */
        [[nodiscard]] virtual bool isMemoized(const std::shared_ptr<const Species>& species,
                                              const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                              const std::shared_ptr<const ConfigurationBase>& goal_configuration) const;

        //! Clears the cache of previously computed motion plans
        void clearCache();

        //! \returns The number of motion plans computed
        [[nodiscard]] inline unsigned int numMotionPlans() const;

        //! \returns The number of times motion planning failed to find a solution
        [[nodiscard]] static unsigned int numFailures();

       protected:
        /*!
         * \brief Returns the previously computed result of a motion planning query if one exists
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The previously computed result of a motion planning query if one exists, nullptr otherwise
         */
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> getMemoized(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) const;

        /*!
         * \brief Computes a motion plan
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The computed motion planning result
         */
        [[nodiscard]] virtual std::shared_ptr<const MotionPlannerQueryResultBase> computeMotionPlan(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) = 0;

        using MemoizationValue = std::tuple<std::shared_ptr<const ConfigurationBase>,
                                            std::shared_ptr<const ConfigurationBase>,
                                            std::shared_ptr<const MotionPlannerQueryResultBase>>;

        std::shared_ptr<const ParametersBase> m_parameters;
        std::shared_ptr<EnvironmentBase> m_environment;
        std::multimap<std::weak_ptr<const Species>, MemoizationValue, std::owner_less<>> m_memoization;
        mutable std::mutex m_mutex;  //!< mutable so that it can be used to lock const functions

        static unsigned int s_num_failures;
    };

    // Inline Functions
    const std::shared_ptr<const ParametersBase>& MotionPlannerBase::parameters() const
    {
        return m_parameters;
    }

    const std::shared_ptr<EnvironmentBase>& MotionPlannerBase::environment() const
    {
        return m_environment;
    }

    unsigned int MotionPlannerBase::numMotionPlans() const
    {
        return m_memoization.size();
    }
}  // namespace grstapse