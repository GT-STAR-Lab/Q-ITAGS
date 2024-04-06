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
// Local
#include "grstapse/common/search/undirected_graph/undirected_graph.hpp"
#include "grstapse/common/utilities/json_extension.hpp"

namespace grstapse
{
    /*!
     * \brief Base class for a search node that corresponds to a vertex in an undirected graph
     *
     * \tparam UndirectedGraphSearchNodeDeriv A derivative of UndirectedGraphSearchNodeBase
     * \tparam BestFirstSearchNodeDeriv A derivative of BestFirstSearchNodeBase
     * \tparam VertexPayload A payload associated with each vertex
     */
    template <template <typename> typename UndirectedGraphSearchNodeDeriv,
              template <typename>
              typename BestFirstSearchNode,
              typename VertexPayload = DummyVertexPayload>
    class UndirectedGraphSearchNodeBase : public BestFirstSearchNode<UndirectedGraphSearchNodeDeriv<VertexPayload>>
    {
        using Base_ = BestFirstSearchNode<UndirectedGraphSearchNodeDeriv<VertexPayload>>;

       public:
        using Graph  = UndirectedGraph<VertexPayload>;
        using Vertex = typename Graph::Vertex;
        using Edge   = typename Graph::Edge;

        //! \returns The vertex this search node represents
        [[nodiscard]] inline const std::shared_ptr<Vertex>& vertex() const
        {
            return m_vertex;
        }

        //! \returns The edge connecting the previous vertex in the search to the underlying vertex
        [[nodiscard]] inline const std::shared_ptr<Edge>& lastEdge() const
        {
            return m_last_edge;
        }

        //! \copydoc SearchNodeBase
        [[nodiscard]] virtual unsigned int hash() const final override
        {
            return m_vertex->id();
        }

        //! \copydoc SearchNodeBase
        [[nodiscard]] nlohmann::json serializeToJson(
            const std::shared_ptr<const ProblemInputs>& problem_inputs) const override
        {
            nlohmann::json j;
            j.push_back({{constants::k_vertex, m_vertex}, {constants::k_last_edge, m_last_edge}});
            for(std::shared_ptr<const UndirectedGraphSearchNodeBase> p = this->parent(); p != nullptr; p = p->parent())
            {
                j.push_back({{constants::k_vertex, p->m_vertex}, {constants::k_last_edge, m_last_edge}});
            }
            std::reverse(j.begin(), j.end());
            return j;
        }

       protected:
        /*!
         * \brief Constructor
         *
         * \param vertex A vertex from an undirected graph
         * \param last_edge The edge connecting the previous vertex in the search to \p vertex
         * \param parent The previous vertex in the search
         */
        explicit UndirectedGraphSearchNodeBase(
            const std::shared_ptr<Vertex>& vertex,
            const std::shared_ptr<Edge>& last_edge                                             = nullptr,
            const std::shared_ptr<const UndirectedGraphSearchNodeDeriv<VertexPayload>>& parent = nullptr)
            : Base_(vertex->id(), parent)
            , m_vertex(vertex)
            , m_last_edge(last_edge)
        {}

        std::shared_ptr<Vertex> m_vertex;
        std::shared_ptr<Edge> m_last_edge;
    };
}  // namespace grstapse