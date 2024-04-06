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
#include "grstapse/geometric_planning/configurations/se2_ompl_configuration.hpp"

namespace grstapse
{
    //! Container for an SE(2) state that represents the initial/terminal configuration for a task
    class Se2StateOmplConfiguration : public Se2OmplConfiguration
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param x The x component of the SE(2) configuration
         * \param y The y component of the SE(2) configuration
         * \param yaw The yaw component of the SE(2) configuration
         */
        Se2StateOmplConfiguration(const float x, const float y, const float yaw);

        //! \returns The x component of the SE(2) configuration
        [[nodiscard]] inline float x() const;

        //! \returns The y component of the SE(2) configuration
        [[nodiscard]] inline float y() const;

        //! \returns The yaw component of the SE(2) configuration
        [[nodiscard]] inline float yaw() const;

        //! \copydoc ConfigurationBase
        [[nodiscard]] bool operator==(const ConfigurationBase& rhs) const final override;

        //! Equality Operator
        [[nodiscard]] bool operator==(const Se2StateOmplConfiguration& rhs) const;

        //! \copydoc ConfigurationBase
        [[nodiscard]] float euclideanDistance(const ConfigurationBase& rhs) const final override;

        //! Euclidean Distance
        [[nodiscard]] float euclideanDistance(const Se2StateOmplConfiguration& rhs) const;

        //! \copydoc OmplConfiguration
        [[nodiscard]] ompl::base::ScopedStatePtr convertToScopedStatePtr(
            const ompl::base::StateSpacePtr& state_space) const final override;
        //! \copydoc OmplConfiguration
        [[nodiscard]] ompl::base::GoalPtr convertToGoalPtr(
            const ompl::base::SpaceInformationPtr& space_information) const final override;

       private:
        float m_x;
        float m_y;
        float m_yaw;
    };

    // Inline Functions
    float Se2StateOmplConfiguration::x() const
    {
        return m_x;
    }

    float Se2StateOmplConfiguration::y() const
    {
        return m_y;
    }

    float Se2StateOmplConfiguration::yaw() const
    {
        return m_yaw;
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const grstapse::Se2StateOmplConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const grstapse::Se2StateOmplConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const grstapse::Se2StateOmplConfiguration>& c);
    };

    template <>
    struct adl_serializer<std::shared_ptr<grstapse::Se2StateOmplConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::Se2StateOmplConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<grstapse::Se2StateOmplConfiguration>& c);
    };
}  // namespace nlohmann