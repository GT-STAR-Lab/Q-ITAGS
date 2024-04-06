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
#include <list>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    // Forward Declarations
    template <typename, typename>
    class VertexEdgeList;
    template <typename, typename>
    class VertexInwardEdgeList;

    /*!
     * \brief Base class for a graph vertex
     *
     * \tparam VertexPayload
     * \tparam EdgePayload
     * \tparam Edge
     */
    // clang-format off
    template <typename VertexPayload,
              typename EdgePayload,
              template <typename, typename, typename> typename Edge>
    // clang-format on
    class VertexBase : public Noncopyable
    {
       public:
        using VertexType        = VertexBase<VertexPayload, EdgePayload, Edge>;
        using VertexPayloadType = VertexPayload;

        using EdgeType        = Edge<VertexPayload, EdgePayload, VertexBase>;
        using EdgePayloadType = EdgePayload;

        using VertexEdgeRange      = VertexEdgeList<VertexType, EdgeType>&;
        using ConstVertexEdgeRange = const VertexEdgeRange;

        using VertexOutwardEdgeList       = std::list<EdgeType>;
        using VertexOutwardEdgeRange      = VertexOutwardEdgeList&;
        using ConstVertexOutwardEdgeRange = const VertexOutwardEdgeRange;
        using VertexOutwardEdgeIterator   = typename VertexOutwardEdgeList::iterator;

        using VertexInwardEdgeRange      = VertexInwardEdgeList<VertexType, EdgeType>&;
        using ConstVertexInwardEdgeRange = const VertexInwardEdgeRange;
        using VertexInwardEdgeIterator   = typename VertexInwardEdgeList<VertexType, EdgeType>::iterator;

        // No Default or Copy Constructor
        //! Move Constructor
        VertexBase(VertexBase&&) noexcept = default;
        //! Destructor
        ~VertexBase() = default;
        // No Copy Assignment
        //! Move Assignment
        VertexBase& operator=(VertexBase&&) noexcept = default;

        //! Constructs a vertex using a \p key
        explicit VertexBase(std::size_t key) noexcept
            : m_key(key)
        {}
        //! Constructs a vertex using a \p key and a \p payload
        VertexBase(std::size_t key, VertexPayload&& payload) noexcept
            : m_key(key)
            , m_payload(std::move(payload))
        {}

        [[nodiscard]] inline std::size_t key() const noexcept
        {
            return m_key;
        }

        //! \returns The payload attached to this vertex
        [[nodiscard]] inline VertexPayloadType& payload() noexcept
        {
            return m_payload;
        }
        //! \returns The payload attached to this vertex
        [[nodiscard]] inline const VertexPayloadType& payload() const noexcept
        {
            return m_payload;
        }

        //! \returns A range of the edges where this vertex is either the source or the target
        [[nodiscard]] inline VertexEdgeRange incidentEdges() noexcept
        {
            return m_edges;
        }
        //! \returns A range of the edges where this vertex is either the source or the target
        [[nodiscard]] inline ConstVertexEdgeRange incidentEdges() const noexcept
        {
            return m_edges;
        }

       protected:
        //! \returns A range of the edges where this vertex is the source
        [[nodiscard]] inline VertexOutwardEdgeRange outwardIncidentEdges() noexcept
        {
            return m_edges.outwardEdges();
        }
        //! \returns A range of the edges where this vertex is the source
        [[nodiscard]] inline ConstVertexOutwardEdgeRange outwardIncidentEdges() const noexcept
        {
            return m_edges.outwardEdges();
        }
        //! \returns A range of the edges where this vertex is the target
        [[nodiscard]] inline VertexInwardEdgeRange inwardIncidentEdges() noexcept
        {
            return m_edges.inwardEdges();
        }
        //! \returns A range of the edges where this vertex is the target
        [[nodiscard]] inline ConstVertexInwardEdgeRange inwardIncidentEdges() const noexcept
        {
            return m_edges.inwardEdges();
        }

        /*!
         * \brief Creates a edge with this vertex as the source
         * \returns An iterator to the created edge
         */
        inline VertexOutwardEdgeIterator createOutwardEdge(VertexType* target)
        {
            return m_edges.createOutwardEdge(this, target);
        }
        /*!
         * \brief Creates a edge with this vertex as the source
         * \returns An iterator to the created edge
         */
        inline VertexOutwardEdgeIterator createOutwardEdge(VertexType* target, EdgePayloadType&& payload)
        {
            return m_edges.createOutwardEdge(this, target, std::forward(payload));
        }
        /*!
         * \brief Gives this vertex a pointer to an edge with this vertex as the target
         * \returns An iterator to the created edge
         */
        inline VertexInwardEdgeIterator createInwardEdge(EdgeType* e)
        {
            return m_edges.createInwardEdge(e);
        }
        /*!
         * \brief Gives this vertex a pointer to an edge with this vertex as the target
         * \returns An iterator to the created edge
         */
        inline VertexInwardEdgeIterator createInwardEdge(VertexOutwardEdgeIterator e)
        {
            return m_edges.createInwardEdge(e);
        }

        //! Decrements the key (Used by GraphBase when a vertex is removed)
        inline void decrementKey() noexcept
        {
            --m_key;
        }

        std::size_t m_key;
        VertexPayloadType m_payload;
        VertexEdgeList<VertexType, EdgeType> m_edges;
    };

}  // namespace grstapse