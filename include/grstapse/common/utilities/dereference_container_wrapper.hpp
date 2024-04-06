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
#include <ranges>

#include <concepts>
// External
// Local
#include "grstapse/common/utilities/std_extension.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    // endregion

    /*!
     * \brief This is a light-weight wrapper for a container of pointers in order to not expose the pointers
     *
     * \note This wrapper cannot modify the internal container (That must be done on the container directly)
     * \todo Reverse iterators?
     */
    // clang-format off
    template <typename Value,
              template <typename> typename ContainerType>
    requires std_ext::Container<ContainerType<Value*>>
    // clang-format on
    class DereferenceContainerWrapper
    {
       public:
        // region Typedefs
        using value_type      = Value;
        using size_type       = typename ContainerType<Value*>::size_type;
        using difference_type = typename ContainerType<Value*>::difference_type;
        using reference       = value_type&;
        using const_reference = const value_type&;
        using pointer         = value_type*;
        using const_pointer   = const value_type*;
        // endregion

        class const_iterator
        {
           public:
            // region Typedefs
            // clang-format off
            using iterator_category = std::conditional_t<std::ranges::random_access_range<ContainerType<Value*>>,
                                                         std::random_access_iterator_tag,
                                                         std::conditional_t<std::ranges::bidirectional_range<ContainerType<Value*>>,
                                                                            std::bidirectional_iterator_tag,
                                                                            std::forward_iterator_tag>>;
            // clang-format on
            using value_type      = typename DereferenceContainerWrapper::value_type;
            using difference_type = typename DereferenceContainerWrapper::difference_type;
            using size_type       = typename DereferenceContainerWrapper::size_type;
            using pointer         = typename DereferenceContainerWrapper::const_pointer;
            using reference       = typename DereferenceContainerWrapper::const_reference;
            // endregion

            //! Constructor
            constexpr const_iterator(typename ContainerType<Value*>::const_iterator iter) noexcept
                : m_internal_iterator(std::move(iter))
            {}

            /*!
             * \brief Default Constructor
             * \note Required for this to be an iterator
             */
            const_iterator() = default;
            /*!
             * \brief Copy Constructor
             * \note Required for this to be an iterator
             */
            constexpr const_iterator(const const_iterator&) noexcept = default;
            /*!
             * Move Constructor
             * \note Required for this to be an iterator
             */
            constexpr const_iterator(const_iterator&&) noexcept = default;
            /*!
             * \brief Destructor
             * \note Required for this to be an iterator
             */
            ~const_iterator() noexcept = default;

            /*!
             * brief Copy Assignment Operator
             * \note Required for this to be an iterator
             */
            constexpr const_iterator& operator=(const const_iterator&) noexcept = default;
            /*!
             * \brief Move Assignment Operator
             * \note Required for this to be an iterator
             */
            constexpr const_iterator& operator=(const_iterator&&) noexcept = default;

            /*!
             * \brief Dereference Operator
             *
             * \note Cannot be noexcept because something else owns the pointer
             * \note Required for this to be an iterator
             */
            constexpr reference operator*() const
            {
                return *(*m_internal_iterator);
            }

            /*!
             * \brief Arrow Operator
             *
             * \note Cannot be noexcept because something else owns the pointer
             * \note Required for this to be an iterator
             */
            constexpr pointer operator->() const
            {
                return *m_internal_iterator;
            }

            /*!
             * \brief Pre-Increment Operator
             * \note Required for this to be an iterator
             */
            constexpr const_iterator& operator++() noexcept
            {
                ++m_internal_iterator;
                return *this;
            }
            /*!
             * \brief Post-Increment Operator
             * \note Required for this to be an iterator
             */
            constexpr const_iterator operator++(int) noexcept
            {
                const_iterator rv(*this);
                ++*this;
                return rv;
            }
            /*!
             * \brief Addition Self Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            constexpr const_iterator& operator+=(difference_type d) noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                m_internal_iterator += d;
                return *this;
            }
            /*!
             * \brief Addition Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr inline const_iterator operator+(difference_type d) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return const_iterator{m_internal_iterator + d};
            }
            /*!
             * \brief Addition Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] friend constexpr inline const_iterator operator+(difference_type d, const_iterator c) noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return c + d;
            }
            /*!
             * \brief Pre-Decrement Operator
             * \note Required for this to be a bidirectional iterator
             */
            // clang-format off
            constexpr const_iterator& operator--() noexcept
            requires (not std::same_as<iterator_category, std::forward_iterator_tag>)
            // clang-format on
            {
                --m_internal_iterator;
                return *this;
            }
            /*!
             * \brief Post-Decrement Operator
             * \note Required for this to be a bidirectional iterator
             */
            // clang-format off
            constexpr const_iterator operator--(int) noexcept
            requires (not std::same_as<iterator_category, std::forward_iterator_tag>)
            // clang-format on
            {
                const_iterator rv(*this);
                --*this;
                return rv;
            }
            /*!
             * \brief Self-Subtraction Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            constexpr const_iterator& operator-=(difference_type d) noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                m_internal_iterator -= d;
                return *this;
            }
            /*!
             * \brief Subtraction Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr inline const_iterator operator-(difference_type d) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return const_iterator{m_internal_iterator - d};
            }
            /*!
             * \brief Subtraction Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr inline difference_type operator-(const_iterator rhs) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return m_internal_iterator - rhs.m_internal_iterator;
            }

            /*!
             * \brief Bracket operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr reference& operator[](difference_type d) const
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return *(*(m_internal_iterator + d));
            }

            /*!
             * \brief Equality Operator
             * \note Required for this to be an iterator
             */
            [[nodiscard]] constexpr bool operator==(const const_iterator& rhs) const noexcept
            {
                return m_internal_iterator == rhs.m_internal_iterator;
            }

            /*!
             * \brief Spaceship Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr auto operator<=>(const const_iterator& rhs) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return m_internal_iterator <=> rhs.m_internal_iterator;
            }

            void swap(const const_iterator& rhs)
            {
                std::swap(m_internal_iterator, rhs.m_internal_iterator);
            }

           private:
            typename ContainerType<Value*>::const_iterator m_internal_iterator;
        };
        class iterator
        {
           public:
            // region Typedefs
            // clang-format off
            using iterator_category = std::conditional_t<std::ranges::random_access_range<ContainerType<Value*>>,
                                                         std::random_access_iterator_tag,
                                                         std::conditional_t<std::ranges::bidirectional_range<ContainerType<Value*>>,
                                                                            std::bidirectional_iterator_tag,
                                                                            std::forward_iterator_tag>>;
            // clang-format on
            using value_type      = typename DereferenceContainerWrapper::value_type;
            using difference_type = typename DereferenceContainerWrapper::difference_type;
            using size_type       = typename DereferenceContainerWrapper::size_type;
            using pointer         = typename DereferenceContainerWrapper::pointer;
            using reference       = typename DereferenceContainerWrapper::reference;
            // endregion

            //! Constructor
            constexpr iterator(typename ContainerType<Value*>::iterator iter) noexcept
                : m_internal_iterator(std::move(iter))
            {}

            /*!
             * \brief Default Constructor
             * \note Required for this to be an iterator
             */
            iterator() = default;
            /*!
             * \brief Copy Constructor
             * \note Required for this to be an iterator
             */
            constexpr iterator(const iterator&) noexcept = default;
            /*!
             * Move Constructor
             * \note Required for this to be an iterator
             */
            constexpr iterator(iterator&&) noexcept = default;
            /*!
             * \brief Destructor
             * \note Required for this to be an iterator
             */
            ~iterator() noexcept = default;

            /*!
             * brief Copy Assignment Operator
             * \note Required for this to be an iterator
             */
            constexpr iterator& operator=(const iterator&) noexcept = default;
            /*!
             * \brief Move Assignment Operator
             * \note Required for this to be an iterator
             */
            constexpr iterator& operator=(iterator&&) noexcept = default;

            /*!
             * \brief Dereference Operator
             *
             * \note Cannot be noexcept because something else owns the pointer
             * \note Required for this to be an iterator
             */
            constexpr reference operator*() const
            {
                return *(*m_internal_iterator);
            }

            /*!
             * \brief Arrow Operator
             *
             * \note Cannot be noexcept because something else owns the pointer
             * \note Required for this to be an iterator
             */
            constexpr pointer operator->() const
            {
                return *m_internal_iterator;
            }

            /*!
             * \brief Pre-Increment Operator
             * \note Required for this to be an iterator
             */
            constexpr iterator& operator++() noexcept
            {
                ++m_internal_iterator;
                return *this;
            }
            /*!
             * \brief Post-Increment Operator
             * \note Required for this to be an iterator
             */
            constexpr iterator operator++(int) noexcept
            {
                iterator rv(*this);
                ++*this;
                return rv;
            }
            /*!
             * \brief Addition Self Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            constexpr iterator& operator+=(difference_type d) noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                m_internal_iterator += d;
                return *this;
            }
            /*!
             * \brief Addition Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr inline iterator operator+(difference_type d) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return iterator{m_internal_iterator + d};
            }
            /*!
             * \brief Addition Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] friend constexpr inline iterator operator+(difference_type d, iterator c) noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return c + d;
            }
            /*!
             * \brief Pre-Decrement Operator
             * \note Required for this to be a bidirectional iterator
             */
            // clang-format off
            constexpr iterator& operator--() noexcept
            requires (not std::same_as<iterator_category, std::forward_iterator_tag>)
            // clang-format on
            {
                --m_internal_iterator;
                return *this;
            }
            /*!
             * \brief Post-Decrement Operator
             * \note Required for this to be a bidirectional iterator
             */
            // clang-format off
            constexpr iterator operator--(int) noexcept
            requires (not std::same_as<iterator_category, std::forward_iterator_tag>)
            // clang-format on
            {
                iterator rv(*this);
                --*this;
                return rv;
            }
            /*!
             * \brief Self-Subtraction Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            constexpr iterator& operator-=(difference_type d) noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                m_internal_iterator -= d;
                return *this;
            }
            /*!
             * \brief Subtraction Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr inline iterator operator-(difference_type d) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return iterator{m_internal_iterator - d};
            }
            /*!
             * \brief Subtraction Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr inline difference_type operator-(iterator rhs) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return m_internal_iterator - rhs.m_internal_iterator;
            }

            /*!
             * \brief Bracket operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr reference& operator[](difference_type d) const
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return *(*(m_internal_iterator + d));
            }

            /*!
             * \brief Equality Operator
             * \note Required for this to be an iterator
             */
            [[nodiscard]] constexpr bool operator==(const iterator& rhs) const noexcept
            {
                return m_internal_iterator == rhs.m_internal_iterator;
            }

            /*!
             * \brief Spaceship Operator
             * \note Required for this to be a random access iterator
             */
            // clang-format off
            [[nodiscard]] constexpr auto operator<=>(const iterator& rhs) const noexcept
            requires std::same_as<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                return m_internal_iterator <=> rhs.m_internal_iterator;
            }

            /*!
             * \brief Swap
             * \note Required for this to be an iterator
             */
            constexpr void swap(const iterator& rhs)
            {
                std::swap(m_internal_iterator, rhs.m_internal_iterator);
            }

           private:
            typename ContainerType<Value*>::iterator m_internal_iterator;
        };

        //! Constructor
        explicit DereferenceContainerWrapper(ContainerType<Value*>* container)
            : m_container(container)
        {}
        // region Special Member Functions
        /*!
         * \brief Default Constructor
         * \note Required to be a Container
         */
        DereferenceContainerWrapper()
            : m_container(nullptr)
        {}
        /*!
         * \brief Copy Constructor
         * \note Required to be a Container
         */
        DereferenceContainerWrapper(const DereferenceContainerWrapper&) noexcept = default;
        /*!
         * \brief Move Constructor
         * \note Required to be a Container
         */
        DereferenceContainerWrapper(DereferenceContainerWrapper&&) noexcept = default;
        /*!
         * \brief Destructor
         * \note Required to be a Container
         */
        ~DereferenceContainerWrapper() = default;
        /*!
         * \brief Copy Assignment Operator
         * \note Required to be a Container
         */
        DereferenceContainerWrapper& operator=(const DereferenceContainerWrapper&) noexcept = default;
        /*!
         * \brief Move Assignment Operator
         * \note Required to be a Container
         */
        DereferenceContainerWrapper& operator=(DereferenceContainerWrapper&&) noexcept = default;
        // endregion

        // clang-format off
        [[nodiscard]] inline reference front()
        requires std_ext::HasFrontBack<ContainerType<Value*>>
        // clang-format on
        {
            return *m_container->front();
        }
        // clang-format off
        [[nodiscard]] inline const_reference front() const
        requires std_ext::HasFrontBack<ContainerType<Value*>>
        // clang-format on
        {
            return *m_container->front();
        }
        // clang-format off
        [[nodiscard]] inline reference back()
        requires std_ext::HasFrontBack<ContainerType<Value*>>
        // clang-format on
        {
            return *m_container->back();
        }
        // clang-format off
        [[nodiscard]] inline const_reference back() const
        requires std_ext::HasFrontBack<ContainerType<Value*>>
        // clang-format on
        {
            return *m_container->back();
        }

        //! \note Required to be a Container
        [[nodiscard]] inline iterator begin()
        {
            return iterator{m_container->begin()};
        }
        //! \note Required to be a Container
        [[nodiscard]] inline const_iterator begin() const
        {
            // The internal m_container may not be const, so we pretend it is through cbegin
            return const_iterator{m_container->cbegin()};
        }
        //! \note Required to be a Container
        [[nodiscard]] inline const_iterator cbegin() const
        {
            return const_iterator{m_container->cbegin()};
        }
        //! \note Required to be a Container
        [[nodiscard]] inline iterator end()
        {
            return iterator{m_container->end()};
        }
        //! \note Required to be a Container
        [[nodiscard]] inline const_iterator end() const
        {
            // The internal m_container may not be const, so we pretend it is through cend
            return const_iterator{m_container->cend()};
        }
        //! \note Required to be a Container
        [[nodiscard]] inline const_iterator cend() const
        {
            return const_iterator{m_container->cend()};
        }

        //! \note Required to be a Container
        [[nodiscard]] inline bool empty() const
        {
            return m_container->empty();
        }
        //! \note Required to be a Container
        [[nodiscard]] inline std::size_t size() const
        {
            return m_container->size();
        }
        //! \note Required to be a Container
        [[nodiscard]] inline std::size_t max_size() const
        {
            return m_container->max_size();
        }

        /*!
         * \brief Equality Operator (Synthesizes !=)
         * \note Required to be a Container
         */
        [[nodiscard]] inline bool operator==(const DereferenceContainerWrapper& rhs) const
        {
            return *m_container == *rhs.m_container;
        }

        //! Space Ship Operator (Synthesizes <, <=, >, >=)
        [[nodiscard]] inline auto operator<=>(const DereferenceContainerWrapper& rhs) const
        {
            return *m_container <=> rhs.m_container;
        }

        /*!
         * \brief Swaps this and rhs
         * \note Required to be a Container
         */
        void swap(DereferenceContainerWrapper& rhs)
        {
            std::swap(m_container, rhs.m_container);
        }

       private:
        ContainerType<Value*>* m_container;
    };

}  // namespace grstapse