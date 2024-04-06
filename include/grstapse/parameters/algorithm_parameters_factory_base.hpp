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
#include <initializer_list>
#include <tuple>
#include <unordered_map>
#include <vector>
// External
#include <nlohmann/json.hpp>
// Local
// endregion

namespace grstapse
{
    // region Forward Declarations
    class ParametersBase;
    // endregion

    /*!
     * \class AlgorithmParametersFactoryBase
     * \brief
     */
    class AlgorithmParametersFactoryBase
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        AlgorithmParametersFactoryBase() = delete;
        //! Copy Constructor
        AlgorithmParametersFactoryBase(const AlgorithmParametersFactoryBase&) = delete;
        //! Move Constructor
        AlgorithmParametersFactoryBase(AlgorithmParametersFactoryBase&&) = delete;
        //! Destructor
        ~AlgorithmParametersFactoryBase() = default;
        //! Copy Assignment Operator
        AlgorithmParametersFactoryBase& operator=(const AlgorithmParametersFactoryBase&) = delete;
        //! Move Assignment Operator
        AlgorithmParametersFactoryBase& operator=(AlgorithmParametersFactoryBase&&) = delete;
        // endregion

        std::shared_ptr<ParametersBase> create(const nlohmann::json& j) const;

        std::shared_ptr<ParametersBase> create(const nlohmann::json& j, const std::string& config_type) const;

       protected:
        explicit AlgorithmParametersFactoryBase(std::string_view base_name);

        void setParent(std::string_view base, std::string_view parent);

        void setParent(std::string_view base, std::initializer_list<std::string_view> parents);
        void setRequired(std::string_view base,
                         std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>> required);
        void setOptional(std::string_view base,
                         std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>> opt);
        void setDefault(std::string_view base, nlohmann::json&& j);

        //! Creates a ParametersBase while validating the individual parameters
        std::shared_ptr<ParametersBase> create(
            nlohmann::json&& j,
            std::multimap<std::string_view, std::string_view>::const_iterator b,
            std::multimap<std::string_view, std::string_view>::const_iterator e) const;

        std::string_view m_base_name;
        std::multimap<std::string_view, std::string_view> m_parents;
        std::unordered_map<std::string_view, std::vector<std::pair<const char* const, nlohmann::json::value_t>>>
            m_requireds;
        std::unordered_map<std::string_view, std::vector<std::pair<const char* const, nlohmann::json::value_t>>>
            m_optionals;
        std::unordered_map<std::string_view, nlohmann::json> m_defaults;
    };  // class AlgorithmParametersFactoryBase
}  // namespace grstapse