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
#include <nlohmann/json.hpp>

namespace grstapse
{
    // Forward Declarations
    enum class ConfigurationType : uint8_t;
    enum class MotionPlannerQueryStatus : uint8_t;

    //! Abstract base class for the result of a motion planner query
    class MotionPlannerQueryResultBase
    {
       public:
        //! \returns The status of the motion plan query
        [[nodiscard]] inline MotionPlannerQueryStatus status() const;

        [[nodiscard]] inline ConfigurationType configurationType() const;

        //! \returns The amount of time needed to execution the motion plan assuming constant \p speed
        [[nodiscard]] inline virtual float duration(const float speed) const;

        //! \returns The length of the motion plan
        [[nodiscard]] virtual float length() const = 0;

       protected:
        //! Constructor
        explicit MotionPlannerQueryResultBase(MotionPlannerQueryStatus status, ConfigurationType configuration_type);

        MotionPlannerQueryStatus m_status;
        ConfigurationType m_configuration_type;
    };

    void to_json(nlohmann::json& j, const MotionPlannerQueryResultBase& r);

    /*!
     * \brief Concept to force a type to derive from MotionPlanningQueryResultBase
     *
     * \tparam T Derivative of MotionPlanningQueryResultBase
     */
    template <typename T>
    concept MotionPlanningQueryResultDeriv = std::derived_from<T, MotionPlannerQueryResultBase>;

    // Inline Functions
    MotionPlannerQueryStatus MotionPlannerQueryResultBase::status() const
    {
        return m_status;
    }

    ConfigurationType MotionPlannerQueryResultBase::configurationType() const
    {
        return m_configuration_type;
    }

    float MotionPlannerQueryResultBase::duration(const float speed) const
    {
        return length() / speed;
    }
}  // namespace grstapse