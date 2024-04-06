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
// External
// Local
#include "grstapse/common/utilities/dereference_container_wrapper.hpp"

namespace grstapse
{
    // Forward Declarations
    template <typename, typename>
    class VertexInwardEdgeList;

    /*!
     * \brief A list of the edges connected to a single vertex
     *
     * \tparam Vertex
     * \tparam Edge
     */
    template <typename Vertex, typename Edge>
    class VertexEdgeList
    {
       public:
        using VertexType = Vertex;

        using EdgeType        = Edge;
        using EdgePayloadType = typename EdgeType::EdgePayloadType;

        using VertexOutwardEdgeList       = std::list<EdgeType>;
        using VertexOutwardEdgeRange      = VertexOutwardEdgeList&;
        using ConstVertexOutwardEdgeRange = const VertexOutwardEdgeRange;
        using VertexOutwardEdgeIterator   = typename VertexOutwardEdgeList::iterator;

        using VertexInwardEdgeList       = DereferenceContainerWrapper<EdgeType, std::list>;
        using VertexInwardEdgeRange      = VertexInwardEdgeList&;
        using ConstVertexInwardEdgeRange = const VertexInwardEdgeRange;
        using VertexInwardEdgeIterator   = typename VertexInwardEdgeList::iterator;

        //! A const_iterator that connects the outward and inward edge lists
        struct const_iterator
        {
            using iterator_category = std::bidirectional_iterator_tag;
            using value_type        = typename std::list<EdgeType>::const_iterator::value_type;
            using difference_type   = typename std::list<EdgeType>::const_iterator::difference_type;
            using size_type         = typename std::list<EdgeType>::const_iterator::size_type;
            using pointer           = typename std::list<EdgeType>::const_iterator::pointer;
            using const_pointer     = typename std::list<EdgeType>::const_iterator::const_pointer;
            using reference         = typename std::list<EdgeType>::const_iterator::reference;
            using const_reference   = typename std::list<EdgeType>::const_iterator::const_reference;

            //! Constructor
            constexpr const_iterator(VertexEdgeList* edge_list,
                                     typename std::list<EdgeType>::const_iterator outward_iter) noexcept
                : m_edge_list(edge_list)
                , m_outward_iterator(std::move(outward_iter))
                , m_inward_iterator(edge_list->m_inward_edges.end())
            {}
            //! Constructor
            constexpr const_iterator(VertexEdgeList* edge_list,
                                     typename std::list<EdgeType*>::const_iterator inward_iter) noexcept
                : m_edge_list(edge_list)
                , m_outward_iterator(edge_list->m_outward_edges.end())
                , m_inward_iterator(std::move(inward_iter))
            {}

            // No Default Constructor
            //! Copy Constructor
            constexpr const_iterator(const const_iterator&) noexcept = default;
            //! Move Constructor
            constexpr const_iterator(const_iterator&&) noexcept = default;
            //! Destructor
            ~const_iterator() noexcept = default;

            //! Copy Assignment Operator
            constexpr const_iterator& operator=(const const_iterator&) noexcept = default;
            //! Move Assignment Operator
            constexpr const_iterator& operator=(const_iterator&&) noexcept = default;

            /*!
             * \brief Dereference Operator
             *
             * \note Cannot be noexcept because some else owns the pointer
             */
            constexpr reference operator*() const
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    return m_outward_iterator.operator*();
                }
                else
                {
                    return m_inward_iterator.operator*();
                }
            }

            /*!
             * \brief Arrow Operator
             *
             * \note Cannot be noexcept because some else owns the pointer
             */
            constexpr pointer operator->() const
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    return m_outward_iterator.operator->();
                }
                else
                {
                    return m_inward_iterator.operator->();
                }
            }

            //! Pre-Increment Operator
            constexpr const_iterator& operator++() noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    ++m_inward_iterator;
                }
                else
                {
                    ++m_outward_iterator;
                }
                return *this;
            }
            //! Post-Increment Operator
            constexpr const_iterator operator++(difference_type) noexcept
            {
                const_iterator rv(*this);
                ++*this;
                return rv;
            }
            //! Addition Self Operator
            constexpr const_iterator& operator+=(difference_type d) noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    m_inward_iterator += d;
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator < d)
                {
                    m_outward_iterator = m_edge_list->m_outward_edges.end();
                    m_inward_iterator  = m_edge_list->m_inward_edges.begin() + d -
                                        (m_edge_list->m_outward_edges.end() - m_outward_iterator);
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator == d)
                {
                    m_outward_iterator = m_edge_list->m_outward_edges.end();
                    m_inward_iterator  = m_edge_list->m_inward_edges.begin();
                }
                else
                {
                    m_outward_iterator += d;
                }
                return *this;
            }
            //! Addition Operator
            [[nodiscard]] constexpr inline const_iterator operator+(difference_type d) const noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    return const_iterator{m_inward_iterator + d};
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator < d)
                {
                    return const_iterator{m_edge_list->m_inward_edges.begin() + d -
                                          (m_edge_list->m_outward_edges.end() - m_outward_iterator)};
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator == d)
                {
                    return const_iterator{m_edge_list->m_inward_edges.begin()};
                }
                else
                {
                    return const_iterator{m_outward_iterator + d};
                }
            }
            //! Pre-Decrement Operator
            constexpr const_iterator& operator--() noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    --m_inward_iterator;
                }
                else
                {
                    --m_outward_iterator;
                }
                return *this;
            }
            //! Post-Increment Operator
            constexpr const_iterator operator--(difference_type) noexcept
            {
                const_iterator rv(*this);
                --*this;
                return rv;
            }
            //! Subtraction Self Operator
            constexpr const_iterator& operator-=(difference_type d) noexcept
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    m_outward_iterator -= d;
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() < d)
                {
                    m_inward_iterator  = m_edge_list->m_inward_edges.begin();
                    m_outward_iterator = m_edge_list->m_outward_edges.end() -
                                         (d - (m_inward_iterator - m_edge_list->m_inward_edges.begin()));
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() == d)
                {
                    m_inward_iterator = m_edge_list->m_inward_edges.begin();
                }
                else
                {
                    m_inward_iterator -= d;
                }
                return *this;
            }
            //! Subtraction Operator
            [[nodiscard]] constexpr inline const_iterator operator-(difference_type d) const noexcept
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    return const_iterator{m_outward_iterator - d};
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() < d)
                {
                    return const_iterator{m_edge_list->m_outward_edges.end() -
                                          (d - (m_inward_iterator - m_edge_list->m_inward_edges.begin()))};
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() == d)
                {
                    return const_iterator{m_edge_list->m_inward_edges.begin()};
                }
                else
                {
                    return const_iterator{m_inward_iterator - d};
                }
            }

            //! Equality Operator
            [[nodiscard]] constexpr bool operator==(const const_iterator& rhs) const noexcept
            {
                return m_outward_iterator == rhs.m_outward_iterator && m_inward_iterator == rhs.m_inward_iterator;
            }

           private:
            VertexEdgeList* m_edge_list;
            typename std::list<EdgeType>::const_iterator m_outward_iterator;
            typename std::list<EdgeType*>::const_iterator m_inward_iterator;
        };
        //! A iterator that connects the outward and inward edge lists
        struct iterator
        {
            using iterator_category = std::bidirectional_iterator_tag;
            using value_type        = typename std::list<EdgeType>::iterator::value_type;
            using difference_type   = typename std::list<EdgeType>::iterator::difference_type;
            using size_type         = typename std::list<EdgeType>::iterator::size_type;
            using pointer           = typename std::list<EdgeType>::iterator::pointer;
            using const_pointer     = typename std::list<EdgeType>::iterator::const_pointer;
            using reference         = typename std::list<EdgeType>::iterator::reference;
            using const_reference   = typename std::list<EdgeType>::iterator::const_reference;

            //! Constructor
            constexpr iterator(VertexEdgeList* edge_list, typename std::list<EdgeType>::iterator outward_iter) noexcept
                : m_edge_list(edge_list)
                , m_outward_iterator(std::move(outward_iter))
                , m_inward_iterator(edge_list->m_inward_edges.end())
            {}
            //! Constructor
            constexpr iterator(VertexEdgeList* edge_list, typename std::list<EdgeType*>::iterator inward_iter) noexcept
                : m_edge_list(edge_list)
                , m_outward_iterator(edge_list->m_outward_edges.end())
                , m_inward_iterator(std::move(inward_iter))
            {}

            // No Default Constructor
            //! Copy Constructor
            constexpr iterator(const iterator&) noexcept = default;
            //! Move Constructor
            constexpr iterator(iterator&&) noexcept = default;
            //! Destructor
            ~iterator() noexcept = default;

            //! Copy Assignment Operator
            constexpr iterator& operator=(const iterator&) noexcept = default;
            //! Move Assignment Operator
            constexpr iterator& operator=(iterator&&) noexcept = default;

            /*!
             * \brief Dereference Operator
             *
             * \note Cannot be noexcept because some else owns the pointer
             */
            constexpr reference operator*() const
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    return m_outward_iterator.operator*();
                }
                else
                {
                    return m_inward_iterator.operator*();
                }
            }

            /*!
             * \brief Arrow Operator
             *
             * \note Cannot be noexcept because some else owns the pointer
             */
            constexpr pointer operator->() const
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    return m_outward_iterator.operator->();
                }
                else
                {
                    return m_inward_iterator.operator->();
                }
            }

            //! Pre-Increment Operator
            constexpr iterator& operator++() noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    ++m_inward_iterator;
                }
                else
                {
                    ++m_outward_iterator;
                }
                return *this;
            }
            //! Post-Increment Operator
            constexpr iterator operator++(difference_type) noexcept
            {
                iterator rv(*this);
                ++*this;
                return rv;
            }
            //! Addition Self Operator
            constexpr iterator& operator+=(difference_type d) noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    m_inward_iterator += d;
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator < d)
                {
                    m_outward_iterator = m_edge_list->m_outward_edges.end();
                    m_inward_iterator  = m_edge_list->m_inward_edges.begin() + d -
                                        (m_edge_list->m_outward_edges.end() - m_outward_iterator);
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator == d)
                {
                    m_outward_iterator = m_edge_list->m_outward_edges.end();
                    m_inward_iterator  = m_edge_list->m_inward_edges.begin();
                }
                else
                {
                    m_outward_iterator += d;
                }
                return *this;
            }
            //! Addition Operator
            [[nodiscard]] constexpr inline iterator operator+(difference_type d) const noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    return iterator{m_inward_iterator + d};
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator < d)
                {
                    return iterator{m_edge_list->m_inward_edges.begin() + d -
                                    (m_edge_list->m_outward_edges.end() - m_outward_iterator)};
                }
                else if constexpr(m_edge_list->m_outward_edges.end() - m_outward_iterator == d)
                {
                    return iterator{m_edge_list->m_inward_edges.begin()};
                }
                else
                {
                    return iterator{m_outward_iterator + d};
                }
            }
            //! Pre-Decrement Operator
            constexpr iterator& operator--() noexcept
            {
                if constexpr(m_outward_iterator == m_edge_list->m_outward_edges.end())
                {
                    --m_inward_iterator;
                }
                else
                {
                    --m_outward_iterator;
                }
                return *this;
            }
            //! Post-Increment Operator
            constexpr iterator operator--(difference_type) noexcept
            {
                iterator rv(*this);
                --*this;
                return rv;
            }
            //! Subtraction Self Operator
            constexpr iterator& operator-=(difference_type d) noexcept
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    m_outward_iterator -= d;
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() < d)
                {
                    m_inward_iterator  = m_edge_list->m_inward_edges.begin();
                    m_outward_iterator = m_edge_list->m_outward_edges.end() -
                                         (d - (m_inward_iterator - m_edge_list->m_inward_edges.begin()));
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() == d)
                {
                    m_inward_iterator = m_edge_list->m_inward_edges.begin();
                }
                else
                {
                    m_inward_iterator -= d;
                }
                return *this;
            }
            //! Subtraction Operator
            [[nodiscard]] constexpr inline iterator operator-(difference_type d) const noexcept
            {
                if constexpr(m_outward_iterator != m_edge_list->m_outward_edges.end())
                {
                    return iterator{m_outward_iterator - d};
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() < d)
                {
                    return iterator{m_edge_list->m_outward_edges.end() -
                                    (d - (m_inward_iterator - m_edge_list->m_inward_edges.begin()))};
                }
                else if constexpr(m_inward_iterator - m_edge_list->m_inward_edges.begin() == d)
                {
                    return iterator{m_edge_list->m_inward_edges.begin()};
                }
                else
                {
                    return iterator{m_inward_iterator - d};
                }
            }

            //! Equality Operator
            [[nodiscard]] constexpr bool operator==(const iterator& rhs) const noexcept
            {
                return m_outward_iterator == rhs.m_outward_iterator && m_inward_iterator == rhs.m_inward_iterator;
            }

           private:
            VertexEdgeList* m_edge_list;
            typename std::list<EdgeType>::iterator m_outward_iterator;
            typename std::list<EdgeType*>::iterator m_inward_iterator;
        };

        //! Default Constructor
        VertexEdgeList() noexcept
            : m_inward_edges(&m_internal_inward_edges)
        {}
        // No Copy Constructor (Cannot copy construct Edge's)
        //! Move Constructor
        VertexEdgeList(VertexEdgeList&&) noexcept = default;
        //! Destructor
        ~VertexEdgeList() = default;
        //! Move Assignment Operator
        VertexEdgeList& operator=(VertexEdgeList&&) noexcept = default;

        //! \returns A range of the edges where this vertex is the source
        [[nodiscard]] inline VertexOutwardEdgeRange outwardEdges() noexcept
        {
            return m_outward_edges;
        }
        //! \returns A range of the edges where this vertex is the source
        [[nodiscard]] inline ConstVertexOutwardEdgeRange outwardEdges() const noexcept
        {
            return m_outward_edges;
        }
        //! \returns A range of the edges where this vertex is the target
        [[nodiscard]] inline VertexInwardEdgeRange inwardEdges() noexcept
        {
            return m_inward_edges;
        }
        //! \returns A range of the edges where this vertex is the target
        [[nodiscard]] inline ConstVertexInwardEdgeRange inwardEdges() const noexcept
        {
            return m_inward_edges;
        }

        //! \returns The begin iterator of this list
        [[nodiscard]] inline iterator begin() noexcept
        {
            return iterator{this, m_outward_edges.begin()};
        }
        //! \returns The begin iterator of this list
        [[nodiscard]] inline const_iterator begin() const noexcept
        {
            return const_iterator{this, m_outward_edges.begin()};
        }
        //! \returns The end iterator of this list
        [[nodiscard]] inline iterator end() noexcept
        {
            return iterator{this, m_inward_edges.end()};
        }
        //! \returns The end iterator of this list
        [[nodiscard]] inline const_iterator end() const noexcept
        {
            return const_iterator{this, m_inward_edges.end()};
        }

        //! \returns Whether this list contains no elements
        [[nodiscard]] inline bool empty() const noexcept
        {
            return m_outward_edges.empty() and m_inward_edges.empty();
        }

        //! \returns The number of elements in this list
        [[nodiscard]] inline std::size_t size() const noexcept
        {
            return m_outward_edges.size() + m_inward_edges.size();
        }

       protected:
        /*!
         * \brief Creates a edge with this vertex as the source
         * \returns An iterator to the created edge
         */
        inline VertexOutwardEdgeIterator createOutwardEdge(VertexType* source, VertexType* target)
        {
            return m_outward_edges.emplace(m_outward_edges.end(), source, target);
        }
        /*!
         * \brief Creates a edge with this vertex as the source
         * \returns An iterator to the created edge
         */
        inline VertexOutwardEdgeIterator createOutwardEdge(VertexType* source,
                                                           VertexType* target,
                                                           EdgePayloadType&& payload)
        {
            return m_outward_edges.emplace(m_outward_edges.end(), source, target, std::move(payload));
        }
        /*!
         * \brief Gives this vertex a pointer to an edge with this vertex as the target
         * \returns An iterator to the created edge
         */
        inline VertexInwardEdgeIterator createInwardEdge(EdgeType* e)
        {
            return m_inward_edges.emplace(e);
        }
        /*!
         * \brief Gives this vertex a pointer to an edge with this vertex as the target
         * \returns An iterator to the created edge
         */
        inline VertexInwardEdgeIterator createInwardEdge(VertexOutwardEdgeIterator e)
        {
            return m_inward_edges.emplace(e);
        }

       private:
        VertexOutwardEdgeList m_outward_edges;  //!< A vertex only contains the edges for which it is the source
        std::list<EdgeType*>
            m_internal_inward_edges;  //!< A vertex contains pointers to the edges for which it is a target
        VertexInwardEdgeList m_inward_edges;

        friend VertexType;
    };

}  // namespace grstapse