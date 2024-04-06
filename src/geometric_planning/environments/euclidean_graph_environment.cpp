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
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"

// Global
#include <ranges>

#include <fstream>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/config.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    EuclideanGraphEnvironment::EuclideanGraphEnvironment()
        : EuclideanGraphEnvironmentBase(EuclideanGraphType::e_singular, false)
    {}

    EuclideanGraphEnvironment::EuclideanGraphEnvironment(bool is_complete)
        : EuclideanGraphEnvironmentBase(EuclideanGraphType::e_singular, is_complete)
    {}

    std::shared_ptr<UndirectedGraphVertex<EuclideanGraphConfiguration>> EuclideanGraphEnvironment::findPossibleVertex(
        const std::shared_ptr<const EuclideanGraphConfiguration>& configuration) const
    {
        if(m_vertices.contains(configuration->id()))
        {
            return m_vertices.at(configuration->id());
        }

        return nullptr;
    }

    std::shared_ptr<UndirectedGraphVertex<EuclideanGraphConfiguration>> EuclideanGraphEnvironment::findVertex(
        const std::shared_ptr<const EuclideanGraphConfiguration>& configuration) const
    {
        if(auto rv = findPossibleVertex(configuration); rv)
        {
            return rv;
        }

        throw createLogicError("Cannot find vertex");
    }

    std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> EuclideanGraphEnvironment::findPossibleEdge(
        const std::shared_ptr<const EuclideanGraphConfiguration>& a,
        const std::shared_ptr<const EuclideanGraphConfiguration>& b) const
    {
        return findPossibleEdge(a->id(), b->id());
    }

    std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> EuclideanGraphEnvironment::findPossibleEdge(
        unsigned int a,
        unsigned int b) const
    {
        if(m_edges.contains(std::pair(a, b)))
        {
            return m_edges.at(std::pair(a, b));
        }
        else if(m_edges.contains(std::pair(b, a)))
        {
            return m_edges.at(std::pair(b, a));
        }

        return nullptr;
    }

    std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> EuclideanGraphEnvironment::findEdge(
        const std::shared_ptr<const EuclideanGraphConfiguration>& a,
        const std::shared_ptr<const EuclideanGraphConfiguration>& b) const
    {
        return findEdge(a->id(), b->id());
    }

    std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> EuclideanGraphEnvironment::findEdge(
        unsigned int a,
        unsigned int b) const
    {
        if(auto rv = findPossibleEdge(a, b); rv)
        {
            return rv;
        }

        throw createLogicError(fmt::format("Cannot find edge ({0:d}, {1:d})", a, b));
    }

    float EuclideanGraphEnvironment::longestPath() const
    {
        auto v = m_edges | std::views::values |
                 std::views::transform(
                     [](const auto& e) -> float
                     {
                         return e->cost();
                     });
        if(m_is_complete)
        {
            return *std::max_element(v.begin(), v.end());
        }
        else
        {
            return std::accumulate(v.begin(), v.end(), 0.0f);
        }
    }

    std::shared_ptr<EuclideanGraphEnvironment> EuclideanGraphEnvironment::shallowCopy() const
    {
        auto rv           = std::make_shared<EuclideanGraphEnvironment>();
        rv->m_is_complete = m_is_complete;
        rv->m_vertices    = m_vertices;
        rv->m_edges       = m_edges;
        return rv;
    }
    std::shared_ptr<EuclideanGraphEnvironment> EuclideanGraphEnvironment::deepCopyVerticesOnly() const
    {
        auto rv = std::make_shared<EuclideanGraphEnvironment>();
        rv->m_vertices.reserve(m_vertices.size());
        for(const auto& [key, vertex]: m_vertices)
        {
            const std::shared_ptr<EuclideanGraphConfiguration>& payload = vertex->payload();
            rv->m_vertices[key] = std::make_shared<UndirectedGraphVertex<EuclideanGraphConfiguration>>(
                key,
                std::make_shared<EuclideanGraphConfiguration>(key, payload->x(), payload->y()));
        }
        return rv;
    }

    void EuclideanGraphEnvironment::internalFromJson(const nlohmann::json& j)
    {
        json_ext::validateJson(j,
                               {{constants::k_vertices, nlohmann::json::value_t::array},
                                {constants::k_edges, nlohmann::json::value_t::array},
                                {constants::k_is_complete, nlohmann::json::value_t::boolean}});
        j.at(constants::k_is_complete).get_to(m_is_complete);
        for(const nlohmann::json& vertex_j: j[constants::k_vertices])
        {
            addVertex(vertex_j[constants::k_id],
                      std::make_shared<EuclideanGraphConfiguration>(vertex_j[constants::k_id],
                                                                    vertex_j[constants::k_x],
                                                                    vertex_j[constants::k_y]));
        }

        for(const nlohmann::json& edge_j: j[constants::k_edges])
        {
            addEdge(edge_j[constants::k_vertex_a].get<unsigned int>(),
                    edge_j[constants::k_vertex_b].get<unsigned int>(),
                    edge_j[constants::k_cost].get<float>());
        }
    }

    void from_json(const nlohmann::json& j, EuclideanGraphEnvironment& environment)
    {
        if(j.contains(constants::k_vertices))
        {
            environment.internalFromJson(j);
        }
        else if(j.contains(constants::k_graph_filepath))
        {
            std::string filepath = "";
            if(j.contains(constants::k_use_data_dir) and j.at(constants::k_use_data_dir).get<bool>())
            {
                filepath += s_data_dir;
            }
            filepath += j[constants::k_graph_filepath].get<std::string>();
            std::ifstream fin(filepath);
            nlohmann::json g;
            fin >> g;
            environment.internalFromJson(g);
        }
        else
        {
            throw createLogicError("Malformed PointGraphEnvironment json");
        }
    }
}  // namespace grstapse