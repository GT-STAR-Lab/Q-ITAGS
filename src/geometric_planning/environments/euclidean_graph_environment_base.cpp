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
#include "grstapse/geometric_planning/environments/euclidean_graph_environment_base.hpp"

// Global
#include <fstream>
// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    EuclideanGraphEnvironmentBase::EuclideanGraphEnvironmentBase(EuclideanGraphType point_graph_type, bool is_complete)
        : GraphEnvironment(GraphType::e_euclidean)
        , m_point_graph_type(point_graph_type)
        , m_is_complete(is_complete)
    {}
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<grstapse::EuclideanGraphEnvironmentBase>
    adl_serializer<std::shared_ptr<grstapse::EuclideanGraphEnvironmentBase>>::from_json(const json& j)
    {
        grstapse::json_ext::validateJson(j,
                                         {{grstapse::constants::k_point_graph_type, nlohmann::json::value_t::string}},
                                         {{grstapse::constants::k_vertices, nlohmann::json::value_t::array},
                                          {grstapse::constants::k_graph_filepath, nlohmann::json::value_t::string}});

        const grstapse::EuclideanGraphType point_graph_type = j.at(grstapse::constants::k_point_graph_type);

        switch(point_graph_type)
        {
            case grstapse::EuclideanGraphType::e_singular:
            {
                return j.get<std::shared_ptr<grstapse::EuclideanGraphEnvironment>>();
            }
            case grstapse::EuclideanGraphType::e_sampled:
            {
                return j.get<std::shared_ptr<grstapse::SampledEuclideanGraphEnvironment>>();
            }
            default:
            {
                throw grstapse::createLogicError(
                    fmt::format("Unknown PointGraphType: {0:s}",
                                j.at(grstapse::constants::k_point_graph_type).get<std::string>()));
            }
        }
    }
}  // namespace nlohmann