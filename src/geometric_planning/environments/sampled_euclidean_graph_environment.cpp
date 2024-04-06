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
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"

// Global
#include <fstream>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/config.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    SampledEuclideanGraphEnvironment::SampledEuclideanGraphEnvironment()
        : EuclideanGraphEnvironmentBase(EuclideanGraphType::e_sampled, false)
    {}

    SampledEuclideanGraphEnvironment::SampledEuclideanGraphEnvironment(bool is_complete)
        : EuclideanGraphEnvironmentBase(EuclideanGraphType::e_sampled, is_complete)
    {}

    const std::shared_ptr<EuclideanGraphEnvironment>& SampledEuclideanGraphEnvironment::graph(unsigned int index) const
    {
        assert(index < m_graphs.size());
        return m_graphs[index];
    }

    void SampledEuclideanGraphEnvironment::addGraph(const std::shared_ptr<EuclideanGraphEnvironment>& g)
    {
        assert(g->isComplete() == m_is_complete);
        m_graphs.push_back(g);
    }

    float SampledEuclideanGraphEnvironment::longestPath() const
    {
        float rv = 0;
        for(const std::shared_ptr<EuclideanGraphEnvironment>& e: m_graphs)
        {
            rv = std::max(rv, e->longestPath());
        }
        return rv;
    }

    void SampledEuclideanGraphEnvironment::internalFromJson(const nlohmann::json& j)
    {
        json_ext::validateJson(j,
                               {{constants::k_vertices, nlohmann::json::value_t::array},
                                {constants::k_edges, nlohmann::json::value_t::array},
                                {constants::k_is_complete, nlohmann::json::value_t::boolean}});
        j.at(constants::k_is_complete).get_to(m_is_complete);
        auto pge_base = std::make_shared<EuclideanGraphEnvironment>();
        for(const nlohmann::json& vertex_j: j[constants::k_vertices])
        {
            pge_base->addVertex(vertex_j[constants::k_id],
                                std::make_shared<EuclideanGraphConfiguration>(vertex_j[constants::k_id],
                                                                              vertex_j[constants::k_x],
                                                                              vertex_j[constants::k_y]));
        }

        for(const nlohmann::json& edges_j: j[constants::k_edges])
        {
            auto pge           = pge_base->deepCopyVerticesOnly();
            pge->m_is_complete = m_is_complete;
            for(const nlohmann::json& edge_j: edges_j)
            {
                pge->addEdge(edge_j[constants::k_vertex_a].get<unsigned int>(),
                             edge_j[constants::k_vertex_b].get<unsigned int>(),
                             edge_j[constants::k_cost].get<float>());
            }
            addGraph(pge);
        }
    }

    void from_json(const nlohmann::json& j, SampledEuclideanGraphEnvironment& environment)
    {
        if(j.contains(constants::k_vertices))
        {
            environment.internalFromJson(j);
        }
        else if(j.contains(constants::k_graph_filepath))
        {
            std::string filepath = "";
            if(j.contains(constants::k_use_data_dir) and j[constants::k_use_data_dir].get<bool>())
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
            throw grstapse::createLogicError("Malformed SampledPointGraphEnvironment json");
        }
    }
}  // namespace grstapse