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

// External
#include <ompl/base/Goal.h>
#include <ompl/base/ScopedState.h>
// Local
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"

namespace grstapse
{
    // Forward Declaration
    enum class OmplGoalType : uint8_t;
    enum class OmplStateSpaceType : uint8_t;

    /*!
     * A configuration used with an OMPL motion planner
     *
     * \see OmplMotionPlanner
     */
    class OmplConfiguration : public ConfigurationBase
    {
       public:
        //! \returns
        [[nodiscard]] inline OmplGoalType goalType() const;

        //! \returns
        [[nodiscard]] inline OmplStateSpaceType stateSpaceType() const;

        //! \returns A scoped state ptr representation of this task configuration
        virtual ompl::base::ScopedStatePtr convertToScopedStatePtr(
            const ompl::base::StateSpacePtr& state_space) const = 0;

        //! \returns A goal ptr representation of this task configuration
        virtual ompl::base::GoalPtr convertToGoalPtr(
            const ompl::base::SpaceInformationPtr& space_information) const = 0;

       protected:
        /*!
         * Constructor
         *
         * \param goal_type
         * \param state_space_type
         */
        OmplConfiguration(OmplGoalType goal_type, OmplStateSpaceType state_space_type);

        OmplGoalType m_ompl_goal_type;
        OmplStateSpaceType m_ompl_state_space_type;
    };
    // Inline Functions
    OmplGoalType OmplConfiguration::goalType() const
    {
        return m_ompl_goal_type;
    }

    OmplStateSpaceType OmplConfiguration::stateSpaceType() const
    {
        return m_ompl_state_space_type;
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const grstapse::OmplConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const grstapse::OmplConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const grstapse::OmplConfiguration>& c);
    };

    template <>
    struct adl_serializer<std::shared_ptr<grstapse::OmplConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::OmplConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<grstapse::OmplConfiguration>& c);
    };
}  // namespace nlohmann