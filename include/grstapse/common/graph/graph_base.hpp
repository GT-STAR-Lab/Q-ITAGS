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
#include <vector>
// External
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/ranges_extension.hpp"
#include "grstapse/common/utilities/std_extension.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    // endregion

    /*!
     * \brief Base class for graphs
     *
     * \tparam Vertex
     * \tparam EdgeList
     * \tparam GraphPayload
     */
    // clang-format off
    template <typename Vertex,
              typename Edge,
              typename GraphPayload>
    // clang-format on
    class GraphBase : public Noncopyable
    {
       public:
        // region typedefs
        using GraphPayloadType = GraphPayload;

        using VertexType          = Vertex;
        using VertexPayloadType   = typename Vertex::VertexPayloadType;
        using VertexRange         = std::vector<Vertex>&;
        using ConstVertexRange    = const VertexRange;
        using VertexIterator      = typename std::vector<Vertex>::iterator;
        using ConstVertexIterator = typename std::vector<Vertex>::const_iterator;

        using EdgeType          = typename Edge::EdgeType;
        using EdgePayloadType   = typename Edge::EdgePayloadType;
        using EdgeRange         = typename Vertex::EdgeList&;
        using ConstEdgeRange    = const EdgeRange;
        using EdgeIterator      = typename Vertex::EdgeList::iterator;
        using ConstEdgeIterator = typename Vertex::EdgeList::const_iterator;
        // endregion
        // region payload
        //! \returns The payload associated with this graph
        [[nodiscard]] inline GraphPayloadType& payload() noexcept
        {
            return m_payload;
        }
        //! \returns The payload associated with this graph
        [[nodiscard]] inline const GraphPayloadType& payload() const noexcept
        {
            return m_payload;
        }
        // endregion
        //  region vertices
        //! \returns The range of vertices
        [[nodiscard]] inline VertexRange vertices() noexcept
        {
            return m_vertices;
        }
        //! \returns The range of vertices
        [[nodiscard]] inline ConstVertexRange vertices() const noexcept
        {
            return m_vertices;
        }
        // endregion
        // region begin
        //! \returns The begin for the range of vertices
        [[nodiscard]] inline VertexIterator begin() noexcept
        {
            return m_vertices.begin();
        }
        //! \returns The begin for the range of vertices
        [[nodiscard]] inline ConstVertexIterator begin() const noexcept
        {
            return m_vertices.begin();
        }
        // endregion
        // region end
        //! \returns The end for the range of vertices
        [[nodiscard]] inline VertexIterator end() noexcept
        {
            return m_vertices.end();
        }
        //! \returns The end for the range of vertices
        [[nodiscard]] inline ConstVertexIterator end() const noexcept
        {
            return m_vertices.end();
        }
        // endregion
        // region findVertex
        //! \returns The specified vertex
        [[nodiscard]] VertexIterator findVertex(std::size_t v) noexcept
        {
            if(v < m_vertices.size())
            {
                return m_vertices.begin() + v;
            }
            return m_vertices.end();
        }
        //! \returns The specified vertex
        [[nodiscard]] ConstVertexIterator findVertex(std::size_t v) const noexcept
        {
            if(v < m_vertices.size())
            {
                return m_vertices.begin() + v;
            }
            return m_vertices.end();
        }
        // endregion
        // region edges
        //! \returns A range of all of the edges in the graph
        [[nodiscard]] virtual EdgeRange edges() = 0;
        //! \returns A ranges of all of the edges in the graph
        [[nodiscard]] virtual ConstEdgeRange edges() const = 0;
        // endregion
        // region clear
        //! Clears the graph
        inline void clear() noexcept
        {
            m_vertices.clear();
        }
        // endregion
       protected:
        // region Special Members
        //! Default Constructor
        GraphBase() = default;
        //! Move Constructor
        GraphBase(GraphBase&&) noexcept = default;
        //! Virtual Destructor
        virtual ~GraphBase() = default;
        //! Move Assignment
        GraphBase& operator=(GraphBase&&) noexcept = default;
        // endregion
        // region Constructors
        //! Constructs a graph with an associated payload for the entire graph
        explicit GraphBase(GraphPayloadType&& payload) noexcept
            : m_payload(std::move(payload))
        {}
        // endregion
        // region init
        //! Initializes the vertices in a graph
        void initVertices(const std::initializer_list<VertexPayloadType>& vertex_list)
        {
            if(vertex_list.size() > 0)
            {
                for(auto& p: vertex_list)
                {
                    createVertex(p);
                }
            }
        }
        //! Initializes the vertices in a graph
        void initVertices(std::initializer_list<VertexPayloadType>&& vertex_list)
        {
            if(vertex_list.size() > 0)
            {
                for(auto& p: vertex_list)
                {
                    createVertex(std::move(p));
                }
            }
        }
        //! Initializes the vertices in a graph
        void initVertices(const std::initializer_list<std::tuple<std::size_t, std::size_t>>& edge_list)
        {
            if(m_vertices.empty())
            {
                std::size_t max_vertex_index =
                    edge_list | ranges_ext::max(
                                    [](const std::tuple<std::size_t, std::size_t>& e) -> std::size_t
                                    {
                                        return std::max(std::get<0>(e), std::get<1>(e));
                                    });
                resizeVertices(max_vertex_index + 1);
            }
        }
        //! Initializes the vertices in a graph
        void initVertices(const std::initializer_list<std::tuple<std::size_t, std::size_t, EdgePayloadType>>& edge_list)
        {
            if(m_vertices.empty())
            {
                std::size_t max_vertex_index =
                    edge_list | ranges_ext::max(
                                    [](const std::tuple<std::size_t, std::size_t>& e) -> std::size_t
                                    {
                                        return std::max(std::get<0>(e), std::get<1>(e));
                                    });
                resizeVertices(max_vertex_index + 1);
            }
        }
        //! Initializes the edges in a graph
        void initEdges(const std::initializer_list<std::tuple<std::size_t, std::size_t>>& edge_list)
        {
            if(edge_list.size() > 0)
            {
                initVertices(edge_list);
                for(auto& e: edge_list)
                {
                    createEdge(e);
                }
            }
        }
        //! Initializes the edges in a graph
        void initEdges(const std::initializer_list<std::tuple<std::size_t, std::size_t, EdgePayloadType>>& edge_list)
        {
            if(edge_list.size() > 0)
            {
                initVertices(edge_list);
                for(auto& [u, v, payload]: edge_list)
                {
                    createEdge(u, v, payload);
                }
            }
        }
        //! Initializes the edges in a graph
        void initEdges(std::initializer_list<std::tuple<std::size_t, std::size_t, EdgePayloadType>>&& edge_list)
        {
            if(edge_list.size() > 0)
            {
                initVertices(edge_list);
                for(auto& [u, v, payload]: edge_list)
                {
                    createEdge(u, v, std::move(payload));
                }
            }
        }

        // endregion
        // region reserveVertices
        //! Reserves space for \p n vertices
        inline void reserveVertices(std::size_t n)
        {
            m_vertices.reserve(n);
        }
        // endregion
        // region resizeVertices
        //! Creates \p n vertices
        inline void resizeVertices(std::size_t n)
        {
            m_vertices.resize(n);
        }
        //! Creates \p n vertices with \p payload
        inline void resizeVertices(std::size_t n, const VertexPayloadType& payload)
        {
            m_vertices.resize(n, payload);
        }
        // endregion
        // region createVertex
        //! Creates a vertex
        [[nodiscard]] inline VertexIterator createVertex()
        {
            return m_vertices.emplace_back(m_vertices.size());
        }
        //! Creates a vertex with a payload
        [[nodiscard]] inline VertexIterator createVertex(VertexPayloadType&& payload)
        {
            return m_vertices.emplace_back(m_vertices.size(), std::move(payload));
        }
        // endregion
        // region removeVertex
        //! Removes a vertex (Also, updates the keys)
        [[nodiscard]] inline VertexIterator removeVertex(std::size_t v)
        {
            return removeVertex(m_vertices.begin() + v);
        }
        //! Removes a vertex (Also, updates the keys)
        [[nodiscard]] VertexIterator removeVertex(VertexIterator v)
        {
            auto rv = m_vertices.erase(v);
            for(auto iter = rv; iter != m_vertices.end(); ++iter)
            {
                iter->decrementKey();
            }
            return rv;
        }
        // endregion
        // region createEdge
        //! Creates an edge from \p u_key to \p v_key
        [[nodiscard]] EdgeIterator createEdge(std::size_t u_key, std::size_t v_key)
        {
            assert(u_key < m_vertices.size() && v_key < m_vertices.size());
            auto& u = m_vertices[u_key];
            auto& v = m_vertices[v_key];
            auto e  = u.createOutwardEdge(&v);
            v.createInwardEdge(e);
        }
        //! Creates an edge from \p u_key to \p v_key with \p payload
        [[nodiscard]] virtual EdgeIterator createEdge(std::size_t u_key, std::size_t v_key, EdgePayloadType&& payload)
        {
            assert(u_key < m_vertices.size() && v_key < m_vertices.size());
            auto& u = m_vertices[u_key];
            auto& v = m_vertices[v_key];
            auto e  = u.createOutwardEdge(&v, std::forward(payload));
            v.createInwardEdge(e);
        }
        //! Creates the specified edge
        [[nodiscard]] virtual EdgeIterator createEdge(const std::tuple<std::size_t, std::size_t>& e)
        {
            return createEdge(std::get<0>(e), std::get<1>(e));
        };
        //! Creates the specified edge with \p payload
        [[nodiscard]] virtual EdgeIterator createEdge(const std::tuple<std::size_t, std::size_t>& e,
                                                      EdgePayloadType&& payload)
        {
            return createEdge(std::get<0>(e), std::get<1>(e), std::forward(payload));
        };
        //! Creates an edge from \p u to \v
        [[nodiscard]] virtual EdgeIterator createEdge(VertexIterator u, VertexIterator v)
        {
            assert(u != m_vertices.end() && v != m_vertices.end());
            auto e = u->createOutwardEdge(v);
            v->createInwardEdge(e);
        }
        //! Creates an edge from \p u to \v with \p payload
        [[nodiscard]] virtual EdgeIterator createEdge(VertexIterator u, VertexIterator v, EdgePayloadType&& payload)
        {
            assert(u != m_vertices.end() && v != m_vertices.end());
            auto e = u->createOutwardEdge(v, std::forward(payload));
            v->createInwardEdge(e);
        }
        // endregion
        // region removeEdge
        //! Removes the edge from \p u to \p v
        [[nodiscard]] virtual EdgeIterator removeEdge(std::size_t u, std::size_t v) = 0;
        //! Removes the edge from \p u to \p v
        [[nodiscard]] virtual EdgeIterator removeEdge(VertexIterator u, VertexIterator v) = 0;
        //! Removes the specified edge
        [[nodiscard]] virtual EdgeIterator removeEdge(const std::tuple<std::size_t, std::size_t>& e) = 0;
        //! Removes the specified edge
        [[nodiscard]] virtual EdgeIterator removeEdge(EdgeIterator e) = 0;
        // endregion
        GraphPayloadType m_payload;
        std::vector<VertexType> m_vertices;
    };

}  // namespace grstapse