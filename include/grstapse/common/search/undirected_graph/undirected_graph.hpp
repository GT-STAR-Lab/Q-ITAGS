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
#include <tuple>
#include <unordered_map>
#include <vector>
// External
#include <fmt/format.h>
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/hash_extension.hpp"

namespace grstapse
{
    // Forward Declarations
    template <typename T>
    class UndirectedGraphVertex;

    //! Dummy Payload for the vertex
    struct DummyVertexPayload
    {};

    //! Deserialize from json
    void to_json(nlohmann::json& j, const DummyVertexPayload& payload);

    /*!
     * \brief An edge from the undirected graph
     *
     * \tparam VertexPayload The type for the payload contained by each vertex
     */
    template <typename VertexPayload = DummyVertexPayload>
    class UndirectedGraphEdge
    {
        using Vertex_ = UndirectedGraphVertex<VertexPayload>;

       public:
        //! Constructor
        UndirectedGraphEdge(const std::shared_ptr<Vertex_>& a, const std::shared_ptr<Vertex_>& b, float cost = 1.0f)
            : m_a(a)
            , m_b(b)
            , m_cost(cost)
        {}

        //! \returns One of vertices at one end of the edge
        [[nodiscard]] inline const std::shared_ptr<Vertex_>& vertexA() const
        {
            return m_a;
        }

        //! \returns The vertex at the other end of the edge
        [[nodiscard]] inline const std::shared_ptr<Vertex_>& vertexB() const
        {
            return m_b;
        }

        //! \returns Whether \p node is one of the vertices that are part of this edge
        [[nodiscard]] inline bool contains(const std::shared_ptr<Vertex_>& node) const
        {
            return node == m_a || node == m_b;
        }

        //! \returns The other node that is part of this edge
        [[nodiscard]] const std::shared_ptr<Vertex_>& other(const std::shared_ptr<Vertex_>& node) const
        {
            if(node == m_a)
            {
                return m_b;
            }
            if(node == m_b)
            {
                return m_a;
            }
            throw createLogicError(fmt::format("Vertex '{0:d}' is not part of this edge", node->id()));
        }

        //! \returns The cost to traverse this edge
        [[nodiscard]] inline float cost() const
        {
            return m_cost;
        }

       private:
        std::shared_ptr<Vertex_> m_a;
        std::shared_ptr<Vertex_> m_b;
        float m_cost;
    };

    /*!
     * \brief A vertex from the undirected graph
     *
     * \tparam VertexPayload The type for the payload contained by each vertex
     */
    template <typename VertexPayload = DummyVertexPayload>
    class UndirectedGraphVertex
    {
        using Edge_ = UndirectedGraphEdge<VertexPayload>;

       public:
        //! Constructor
        UndirectedGraphVertex(unsigned int id, const std::shared_ptr<VertexPayload>& payload = nullptr)
            : m_id(id)
            , m_payload(payload)
        {}

        //! \returns The identifier for this vertex
        [[nodiscard]] inline unsigned int id() const
        {
            return m_id;
        }

        //! \returns The number of edges this vertex is a part of
        [[nodiscard]] inline unsigned int edgeDegree() const
        {
            return m_edges.size();
        }

        //! \returns A list of edges this vertex is a part of
        [[nodiscard]] inline const std::vector<std::shared_ptr<Edge_>>& edges() const
        {
            return m_edges;
        }

        [[nodiscard]] std::shared_ptr<Edge_> edgeTo(const std::shared_ptr<UndirectedGraphVertex>& rhs) const
        {
            const unsigned int rhs_id = rhs->id();
            for(const std::shared_ptr<Edge_>& edge: m_edges)
            {
                const unsigned int a_id = edge->vertexA()->id();
                const unsigned int b_id = edge->vertexB()->id();
                if((a_id == m_id && b_id == rhs_id) || (b_id == m_id && a_id == rhs_id))
                {
                    return edge;
                }
            }
            return nullptr;
        }

        /*!
         * \returns The payload associated with this vertex
         *
         * \note If this vertex contains the dummy payload then this function does not exist
         */
        std::enable_if_t<!std::is_same_v<VertexPayload, DummyVertexPayload>, const std::shared_ptr<VertexPayload>&>
        payload() const
        {
            return m_payload;
        }

        //! Adds an edge that this vertex is a part of
        void addEdge(const std::shared_ptr<Edge_>& edge)
        {
            m_edges.push_back(edge);
        }

       private:
        unsigned int m_id;
        std::vector<std::shared_ptr<Edge_>> m_edges;
        std::shared_ptr<VertexPayload> m_payload;
    };

    /*!
     * \brief An undirected graph
     *
     * \tparam VertexPayload The type for the payload contained by each vertex
     */
    template <typename VertexPayload = DummyVertexPayload>
    class UndirectedGraph
    {
       public:
        using Vertex = UndirectedGraphVertex<VertexPayload>;
        using Edge   = UndirectedGraphEdge<VertexPayload>;

        //! Constructor
        UndirectedGraph() = default;

        //! Adds a vertex to the graph
        const std::shared_ptr<Vertex>& addVertex(unsigned int id,
                                                 const std::shared_ptr<VertexPayload>& payload = nullptr)
        {
            if(m_vertices.contains(id))
            {
                throw createLogicError(fmt::format("Vertex with id '{0:d}' already exists", id));
            }
            auto vertex    = std::make_shared<Vertex>(id, payload);
            m_vertices[id] = vertex;
            return m_vertices[id];
        }

        //! \returns A map of vertices in this graph
        [[nodiscard]] inline const std::unordered_map<unsigned int, std::shared_ptr<Vertex>>& vertices() const
        {
            return m_vertices;
        }

        //! \returns The number of vertices in this graph
        [[nodiscard]] inline unsigned int numVertices() const
        {
            return m_vertices.size();
        }

        //! Adds an edge to the graph
        const std::shared_ptr<Edge>& addEdge(unsigned int a, unsigned int b, float cost = 1.0f)
        {
            if(!m_vertices.contains(a))
            {
                throw createLogicError(fmt::format("Vertex with id '{0:d}' does not exist", a));
            }
            if(!m_vertices.contains(b))
            {
                throw createLogicError(fmt::format("Vertex with id '{0:d}' does not exist", b));
            }

            return addEdge(m_vertices[a], m_vertices[b], cost);
        }

        //! Adds an edge to the graph
        const std::shared_ptr<Edge>& addEdge(const std::shared_ptr<Vertex>& a,
                                             const std::shared_ptr<Vertex>& b,
                                             float cost = 1.0f)
        {
            auto edge = std::make_shared<Edge>(a, b, cost);
            a->addEdge(edge);
            b->addEdge(edge);
            m_edges[std::pair(a->id(), b->id())] = edge;
            return m_edges[std::pair(a->id(), b->id())];
        }

        //! \returns A map of edges in this graph
        [[nodiscard]] inline const std::unordered_map<std::pair<unsigned int, unsigned int>, std::shared_ptr<Edge>>&
        edges() const
        {
            return m_edges;
        }

        //! \returns The number of edges in this graph
        [[nodiscard]] inline unsigned int numEdges() const
        {
            return m_edges.size();
        }

       protected:
        std::unordered_map<unsigned int, std::shared_ptr<Vertex>> m_vertices;
        std::unordered_map<std::pair<unsigned int, unsigned int>, std::shared_ptr<Edge>> m_edges;
    };

}  // namespace grstapse

namespace nlohmann
{
    //! Custom json serialization for grstapse::UndirectedGraphVertex
    template <typename T>
    struct adl_serializer<grstapse::UndirectedGraphVertex<T>>
    {
        static void to_json(json& j, const grstapse::UndirectedGraphVertex<T>& vertex)
        {
            j = vertex.payload();
        }
    };

    //! Custom json serialization for grstapse::UndirectedGraphEdge
    template <typename T>
    struct adl_serializer<grstapse::UndirectedGraphEdge<T>>
    {
        static void to_json(json& j, const grstapse::UndirectedGraphEdge<T>& edge)
        {
            j = {{grstapse::constants::k_cost, edge.cost()},
                 {grstapse::constants::k_vertex_a, edge.vertexA()->id()},
                 {grstapse::constants::k_vertex_b, edge.vertexB()->id()}};
        }
    };
}  // namespace nlohmann