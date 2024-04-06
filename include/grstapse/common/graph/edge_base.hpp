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
#include <vector>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    /*!
     * \brief A base class for a graph edge
     *
     * \tparam VertexPayload
     * \tparam EdgePayload
     * \tparam Vertex
     */
    // clang-format off
    template <typename VertexPayload,
              typename EdgePayload,
              template <typename, typename, typename> typename Vertex>
    // clang-format on
    class EdgeBase : public Noncopyable
    {
       public:
        using VertexType          = Vertex<VertexPayload, EdgePayload, EdgeBase>;
        using VertexIterator      = typename std::vector<VertexType>::iterator;
        using ConstVertexIterator = typename std::vector<VertexType>::const_iterator;

        using EdgeType        = EdgeBase<VertexPayload, EdgePayload, Vertex>;
        using EdgePayloadType = EdgePayload;

        // No Default or Copy Constructor
        //! Move Constructor
        EdgeBase(EdgeBase&&) noexcept = default;
        //! Destructor
        ~EdgeBase() = default;
        // No Copy Assignment
        //! Move Assignment Operator
        EdgeBase& operator=(EdgeBase&&) = default;

        //! Constructs an from two vertices
        EdgeBase(VertexType* source, VertexType* target)
            : m_source(source)
            , m_target(source)
        {}
        //! Constructs an from two vertices and a payload
        EdgeBase(VertexType* source, VertexType* target, EdgePayloadType&& payload)
            : m_source(source)
            , m_target(source)
            , m_payload(std::move(payload))
        {}

        //! \returns The payload for this edge
        [[nodiscard]] inline EdgePayloadType& payload() noexcept
        {
            return m_payload;
        }
        //! \returns The payload for this edge
        [[nodiscard]] inline const EdgePayloadType& payload() const noexcept
        {
            return m_payload;
        }

        //! \returns A pointer to the source vertex of this edge
        [[nodiscard]] inline VertexType* source() noexcept
        {
            return m_source;
        }
        //! \returns A pointer to the source vertex of this edge
        [[nodiscard]] inline const VertexType* source() const noexcept
        {
            return m_source;
        }

        //! \returns Whether \p v is the source of this edge
        [[nodiscard]] inline bool isSource(std::size_t v) const
        {
            return m_source->key() == v;
        }
        //! \returns Whether \p v is the source of this edge
        [[nodiscard]] inline bool isSource(VertexIterator v)
        {
            return *m_source == *v;
        }
        //! \returns Whether \p v is the source of this edge
        [[nodiscard]] inline bool isSource(ConstVertexIterator v) const
        {
            return *m_source == *v;
        }

        //! \returns A pointer to the target vertex of this edge
        [[nodiscard]] inline VertexType* target() noexcept
        {
            return m_target;
        }

        //! \returns A pointer to the target vertex of this edge
        [[nodiscard]] inline const VertexType* target() const noexcept
        {
            return m_target;
        }
        //! \returns Whether \p v is the target of this edge
        [[nodiscard]] inline bool isTarget(std::size_t v) const
        {
            return m_target->key() == v;
        }
        //! \returns Whether \p v is the target of this edge
        [[nodiscard]] bool isTarget(VertexIterator v)
        {
            return *m_target == *v;
        }
        //! \returns Whether \p v is the target of this edge
        [[nodiscard]] bool isTarget(ConstVertexIterator v) const
        {
            return *m_target == *v;
        }

        /*!
         * \returns If \p v is one of the vertices this function returns the other vertex; otherwise it returns a
         *          nullptr
         */
        [[nodiscard]] VertexType* other(std::size_t v)
        {
            if(isSource(v))
            {
                return m_target;
            }
            else if(isTarget(v))
            {
                return m_source;
            }
            else
            {
                return nullptr;
            }
        }

        /*!
         * \returns If \p v is one of the vertices this function returns the other vertex; otherwise it returns a
         *          nullptr
         */
        [[nodiscard]] const VertexType* other(std::size_t v) const
        {
            if(isSource(v))
            {
                return m_target;
            }
            else if(isTarget(v))
            {
                return m_source;
            }
            else
            {
                return nullptr;
            }
        }
        //! \returns The vertex opposite \p v on this edge if it exists; otherwise nullptr
        [[nodiscard]] VertexType* other(VertexIterator v)
        {
            if(isSource(v))
            {
                return m_target;
            }
            else if(isTarget(v))
            {
                return m_source;
            }
            else
            {
                return nullptr;
            }
        }
        //! \returns The vertex opposite \p v on this edge if it exists; otherwise nullptr
        [[nodiscard]] const VertexType* other(ConstVertexIterator v) const
        {
            if(isSource(v))
            {
                return m_target;
            }
            else if(isTarget(v))
            {
                return m_source;
            }
            else
            {
                return nullptr;
            }
        }

       protected:
        VertexType* m_source;
        VertexType* m_target;
        EdgePayloadType m_payload;
    };

}  // namespace grstapse