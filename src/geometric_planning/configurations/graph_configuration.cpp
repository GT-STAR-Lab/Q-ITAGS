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
#include "grstapse/geometric_planning/configurations/graph_configuration.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    GraphConfiguration::GraphConfiguration(GraphType graph_type, unsigned int id)
        : ConfigurationBase(ConfigurationType::e_graph)
        , m_graph_type(graph_type)
        , m_id(id)
    {}
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<const grstapse::GraphConfiguration>
    adl_serializer<std::shared_ptr<const grstapse::GraphConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<grstapse::GraphConfiguration>>::from_json(j);
    }

    std::shared_ptr<grstapse::GraphConfiguration>
    adl_serializer<std::shared_ptr<grstapse::GraphConfiguration>>::from_json(const json& j)
    {
        grstapse::json_ext::validateJson(j, {{grstapse::constants::k_graph_type, nlohmann::json::value_t::string}});
        const grstapse::GraphType type = j.at(grstapse::constants::k_graph_type);

        switch(type)
        {
            case grstapse::GraphType::e_euclidean:
            {
                return j.get<std::shared_ptr<grstapse::EuclideanGraphConfiguration>>();
            }
            case grstapse::GraphType::e_grid:
            {
                // TODO(Andrew)
                throw grstapse::createLogicError("Not implemented");
            }
            default:
            {
                throw grstapse::createLogicError(
                    fmt::format("Unknown GraphType: {0:s}",
                                j.at(grstapse::constants::k_graph_type).get<std::string>()));
            }
        }
    }
    void adl_serializer<std::shared_ptr<const grstapse::GraphConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<const grstapse::GraphConfiguration>& c)
    {
        switch(c->graphType())
        {
            case grstapse::GraphType::e_euclidean:
            {
                j = std::dynamic_pointer_cast<const grstapse::EuclideanGraphConfiguration>(c);
                return;
            }
            case grstapse::GraphType::e_grid:
            {
                // TODO(Andrew)
                throw grstapse::createLogicError("Not implemented");
            }
            default:
            {
                throw grstapse::createLogicError("Unknown GraphType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<grstapse::GraphConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<grstapse::GraphConfiguration>& c)
    {
        adl_serializer<std::shared_ptr<const grstapse::GraphConfiguration>>::to_json(j, c);
    }
}  // namespace nlohmann