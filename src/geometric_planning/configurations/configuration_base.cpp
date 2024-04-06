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
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/graph_configuration.hpp"
#include "grstapse/geometric_planning/configurations/ompl_configuration.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    ConfigurationBase::ConfigurationBase(ConfigurationType type)
        : m_configuration_type(type)
    {}
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<const grstapse::ConfigurationBase>
    adl_serializer<std::shared_ptr<const grstapse::ConfigurationBase>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<grstapse::ConfigurationBase>>::from_json(j);
    }

    std::shared_ptr<grstapse::ConfigurationBase>
    adl_serializer<std::shared_ptr<grstapse::ConfigurationBase>>::from_json(const json& j)
    {
        grstapse::json_ext::validateJson(
            j,
            {{grstapse::constants::k_configuration_type, nlohmann::json::value_t::string}});

        const grstapse::ConfigurationType configuration_type = j.at(grstapse::constants::k_configuration_type);
        switch(configuration_type)
        {
            case grstapse::ConfigurationType::e_ompl:
            {
                return j.get<std::shared_ptr<grstapse::OmplConfiguration>>();
            }
            case grstapse::ConfigurationType::e_graph:
            {
                return j.get<std::shared_ptr<grstapse::GraphConfiguration>>();
            }
            default:
            {
                throw grstapse::createLogicError(
                    fmt::format("Unknown ConfigurationType: {0:s}",
                                j.at(grstapse::constants::k_configuration_type).get<std::string>()));
            }
        }
    }

    void adl_serializer<std::shared_ptr<const grstapse::ConfigurationBase>>::to_json(
        json& j,
        const std::shared_ptr<const grstapse::ConfigurationBase>& c)
    {
        switch(c->configurationType())
        {
            case grstapse::ConfigurationType::e_ompl:
            {
                j = std::dynamic_pointer_cast<const grstapse::OmplConfiguration>(c);
                return;
            }
            case grstapse::ConfigurationType::e_graph:
            {
                j = std::dynamic_pointer_cast<const grstapse::GraphConfiguration>(c);
                return;
            }
            default:
            {
                throw grstapse::createLogicError("Unknown ConfigurationType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<grstapse::ConfigurationBase>>::to_json(
        json& j,
        const std::shared_ptr<grstapse::ConfigurationBase>& c)
    {
        adl_serializer<std::shared_ptr<const grstapse::ConfigurationBase>>::to_json(j, c);
    }
}  // namespace nlohmann