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
#include "grstapse/parameters/algorithm_parameters_factory_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/parameters/parameters_base.hpp"

namespace grstapse
{
    AlgorithmParametersFactoryBase::AlgorithmParametersFactoryBase(std::string_view base_name)
        : m_base_name(base_name)
    {}

    void AlgorithmParametersFactoryBase::setParent(std::string_view base, std::string_view parent)
    {
        m_parents.emplace(base, parent);
    }

    void AlgorithmParametersFactoryBase::setParent(std::string_view base,
                                                   std::initializer_list<std::string_view> parents)
    {
        for(std::string_view parent: parents)
        {
            m_parents.emplace(base, parent);
        }
    }

    void AlgorithmParametersFactoryBase::setRequired(
        std::string_view base,
        std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>> required)
    {
        m_requireds.emplace(base, required);
    }

    void AlgorithmParametersFactoryBase::setOptional(
        std::string_view base,
        std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>> opt)
    {
        m_optionals.emplace(base, opt);
    }

    void AlgorithmParametersFactoryBase::setDefault(std::string_view base, nlohmann::json&& j)
    {
        m_defaults.emplace(base, std::move(j));
    }

    std::shared_ptr<ParametersBase> AlgorithmParametersFactoryBase::create(const nlohmann::json& j) const
    {
        json_ext::validateJson(j, {{constants::k_config_type, nlohmann::json::value_t::string}});
        const std::string config_type = j.at(constants::k_config_type).get<std::string>();

        if(!m_requireds.contains(config_type))
        {
            throw createLogicError(fmt::format("Unknown Config Type '{}'", config_type));
        }
        json_ext::validateJson(j, m_requireds.at(config_type), m_optionals.at(config_type));
        return create(j, config_type);
    }

    std::shared_ptr<ParametersBase> AlgorithmParametersFactoryBase::create(const nlohmann::json& j,
                                                                           const std::string& config_type) const
    {
        nlohmann::json j2 = j;
        for(const auto& [key, val]: m_defaults.at(config_type).items())
        {
            if(not j2.contains(key))
            {
                j2[key] = val;
            }
        }
        if(config_type == m_base_name)
        {
            return std::make_shared<ParametersBase>(std::move(j2), ParametersBase::s_this_is_private_tag);
        }
        if(!m_parents.contains(config_type))
        {
            throw createLogicError(
                fmt::format("Config Type '{}' is not the base, but also doesn't have a parent", config_type));
        }
        auto [b, e] = m_parents.equal_range(config_type);
        return create(std::move(j2), b, e);
    }

    std::shared_ptr<ParametersBase> AlgorithmParametersFactoryBase::create(
        nlohmann::json&& j,
        std::multimap<std::string_view, std::string_view>::const_iterator b,
        std::multimap<std::string_view, std::string_view>::const_iterator e) const
    {
        for(auto iter = b; iter != e; ++iter)
        {
            const std::string_view config_type = iter->second;
            if(!m_requireds.contains(config_type))
            {
                throw createLogicError(fmt::format("Unknown Config Type '{}'", config_type));
            }
            json_ext::validateJson(j, m_requireds.at(config_type), m_optionals.at(config_type));
            for(const auto& [key, val]: m_defaults.at(config_type).items())
            {
                if(not j.contains(key))
                {
                    j[key] = val;
                }
            }
            if(config_type == m_base_name)
            {
                return std::make_shared<ParametersBase>(std::move(j), ParametersBase::s_this_is_private_tag);
            }
            if(!m_parents.contains(config_type))
            {
                throw createLogicError(
                    fmt::format("Config Type '{}' is not the base, but also doesn't have a parent", config_type));
            }
            auto [b, e] = m_parents.equal_range(config_type);
            return create(std::move(j), b, e);
        }
        throw createLogicError("Shouldn't be possible to get here");
    }
}  // namespace grstapse