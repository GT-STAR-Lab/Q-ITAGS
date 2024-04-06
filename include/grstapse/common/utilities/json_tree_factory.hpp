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
// Global
#include <functional>
// External
#include <fmt/format.h>
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
// endregion

namespace grstapse
{
    /*!
     * \class JsonTreeFactory
     * \brief
     */
    template <typename T>
    class JsonTreeFactory
    {
       public:
        // region Special Member Functions
       private:
        //! Default Constructor
        JsonTreeFactory() = default;

       public:
        //! Copy Constructor
        JsonTreeFactory(const JsonTreeFactory&) = delete;
        //! Move Constructor
        JsonTreeFactory(JsonTreeFactory&&) noexcept = delete;
        //! Destructor
        ~JsonTreeFactory() = default;
        //! Copy Assignment Operator
        JsonTreeFactory& operator=(const JsonTreeFactory&) = delete;
        //! Move Assignment Operator
        JsonTreeFactory& operator=(JsonTreeFactory&&) noexcept = delete;
        // endregion

        static JsonTreeFactory& instance()
        {
            static JsonTreeFactory factory;
            return factory;
        }

        [[nodiscard]] std::shared_ptr<T> create(const nlohmann::json& j) const
        {
            json_ext::validateJson(j, {{constants::k_config_type, nlohmann::json::value_t::string}});
            const std::string config_type = j.at(constants::k_config_type).get<std::string>();
            if(auto iter = m_factory.find(config_type); iter != m_factory.end())
            {
                return iter->second(j);
            }
            throw createLogicError(fmt::format("Unknown config type: '{0:s}'", config_type));
        }

        inline void set(std::string_view key, std::function<std::shared_ptr<T>(const nlohmann::json&)> f)
        {
            m_factory.emplace(key, f);
        }

       private:
        std::unordered_map<std::string_view, std::function<std::shared_ptr<T>(const nlohmann::json&)>> m_factory;
    };  // class ConfigFactory

}  // namespace grstapse