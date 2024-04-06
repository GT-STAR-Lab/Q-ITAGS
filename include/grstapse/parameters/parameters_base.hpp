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
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    // endregion

    /*!
     * \class ConfigBase
     * \brief Base class for containers for parameters for various algorithms
     */
    class ParametersBase : public Noncopyable
    {
        //! Tag to make a protected constructor that can still be used with std::make_shared
        constexpr static const struct ThisIsPrivateTag
        {
        } s_this_is_private_tag = ThisIsPrivateTag{};

       public:
        // region Special Member Functions
        //! Default Constructor
        ParametersBase() = delete;
        //! Copy Constructor
        ParametersBase(const ParametersBase&) = delete;
        //! Move Constructor
        ParametersBase(ParametersBase&&) noexcept = default;
        //! Destructor
        ~ParametersBase() = default;
        //! Copy Assignment Operator
        ParametersBase& operator=(const ParametersBase&) = delete;
        //! Move Assignment Operator
        ParametersBase& operator=(ParametersBase&&) noexcept = default;
        // endregion

        //! Copy json Constructor
        explicit ParametersBase(const nlohmann::json& config, const ThisIsPrivateTag&)
            : m_internal(config)
        {}

        //! Move json Constructor
        explicit ParametersBase(nlohmann::json&& config, const ThisIsPrivateTag&)
            : m_internal(std::move(config))
        {}

        //! \returns Whether the specified field is contained within this config
        [[nodiscard]] virtual inline bool contains(const char* key) const
        {
            return m_internal.contains(key);
        }

        //! \returns Whether the specified field is contained within this config
        [[nodiscard]] virtual inline bool contains(const std::string& key) const
        {
            return m_internal.contains(key);
        }

        //! \returns The value associated with the specified field
        template <typename T>
        [[nodiscard]] T get(const char* key) const
        {
            return m_internal.at(key).get<T>();
        }

        //! \returns The value associated with the specified field
        template <typename T>
        [[nodiscard]] T get(const std::string& key) const
        {
            return m_internal.at(key).get<T>();
        }

        //! Assigns the value in the specified field to \p v
        template <typename T>
        void get_to(const char* key, T& v) const
        {
            m_internal.at(key).get_to<T>(v);
        }

        //! Assigns the value in the specified field to \p v
        template <typename T>
        void get_to(const std::string& key, T& v) const
        {
            m_internal.at(key).get_to<T>(v);
        }

       protected:
        nlohmann::json m_internal;

        friend class AlgorithmParametersFactoryBase;
    };  // class ConfigBase
}  // namespace grstapse