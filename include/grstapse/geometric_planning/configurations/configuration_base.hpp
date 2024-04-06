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
#include <nlohmann/json.hpp>

namespace grstapse
{
    // Forward Declarations
    enum class ConfigurationType : uint8_t;

    /*!
     * \brief Abstract base class for the initial/terminal configuration for a task and the initial configuration for a
     * robot
     *
     * \see Task
     * \see Robot
     */
    class ConfigurationBase
    {
       public:
        //! \returns The euclidean distance to \p rhs
        [[nodiscard]] virtual float euclideanDistance(const ConfigurationBase& rhs) const = 0;

        //! Equality operator
        [[nodiscard]] virtual bool operator==(const ConfigurationBase& rhs) const = 0;

        //! \returns
        [[nodiscard]] inline ConfigurationType configurationType() const;

       protected:
        //! Constructor
        explicit ConfigurationBase(ConfigurationType type);

        ConfigurationType m_configuration_type;
    };

    /*!
     * \brief Concept to force a type to derive from ConfigurationBase
     *
     * \tparam T Derivative of ConfigurationBase
     */
    template <typename T>
    concept ConfigurationDeriv = std::derived_from<T, ConfigurationBase>;

    // Inline Functions
    ConfigurationType ConfigurationBase::configurationType() const
    {
        return m_configuration_type;
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const grstapse::ConfigurationBase>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const grstapse::ConfigurationBase> from_json(const json& j);
        //! to_json
        static void to_json(json& j, const std::shared_ptr<const grstapse::ConfigurationBase>& c);
    };

    template <>
    struct adl_serializer<std::shared_ptr<grstapse::ConfigurationBase>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::ConfigurationBase> from_json(const json& j);
        //! to_json
        static void to_json(json& j, const std::shared_ptr<grstapse::ConfigurationBase>& c);
    };
}  // namespace nlohmann