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
#include <exception>
#include <tuple>
// External
// Local
// endregion

namespace grstapse
{
    // region Forward Declarations
    template <typename Value, std::ranges::range... Containers>
    class ConcatenateContainerWrapper;
    // endregion

    namespace detail
    {
        template <typename... Containers>
        constexpr static bool allRandomAccess()
        {
            return (std::is_same_v<typename Containers::iterator::iterator_category, std::random_access_iterator_tag> &&
                    ...);
        }

        template <typename... Containers>
        constexpr static bool allRandomAccessOrBidirectional()
        {
            return (
                (std::is_same_v<typename Containers::iterator::iterator_category, std::random_access_iterator_tag> ||
                 std::is_same_v<typename Containers::iterator::iterator_category,
                                std::bidirectional_iterator_tag>)&&...);
        }

        template <typename T, typename... Containers>
        constexpr static bool possibleIterator()
        {
            return (std::is_same_v<typename Containers::iterator, T> || ...);
        }

        template <typename Value, typename... Containers>
        constexpr static bool IsSameValueContainers()
        {
            return (std::is_same_v<typename Containers::value_type, Value> && ...);
        }

        template <typename... Containers>
        constexpr static bool allContainers()
        {
            return ((std::ranges::range<Containers> && not std::ranges::view<Containers>)&&...);
        }

        template <typename Value, typename... Containers>
        class ConcatenateContainerWrapper_iterator
        {
            static_assert(sizeof...(Containers) > 2);
            constexpr static std::size_t s_num_containers = sizeof...(Containers);
            constexpr static std::size_t s_last_index     = sizeof...(Containers) - 1;

           public:
            // clang-format off
            using iterator_category = std::conditional_t<allRandomAccess<Containers...>(),
                                                         std::random_access_iterator_tag,
                                                         std::conditional_t<allRandomAccessOrBidirectional<Containers...>(),
                                                                            std::bidirectional_iterator_tag,
                                                                            std::forward_iterator_tag>>;
            // clang-format on
            using value_type = Value;
            using size_type  = std::size_t;  //!< This is the standard size_type, so we make an assumption here
            using difference_type =
                std::ptrdiff_t;  //!< This is the standard difference_type, so we make an assumption here
            using reference = value_type&;

            // No Default Constructor
            //! Copy Constructor
            ConcatenateContainerWrapper_iterator(const ConcatenateContainerWrapper_iterator&) noexcept = default;
            //! Move Constructor
            ConcatenateContainerWrapper_iterator(ConcatenateContainerWrapper_iterator&&) noexcept = default;
            //! Destructor
            ~ConcatenateContainerWrapper_iterator() = default;
            //! Copy Assignment Operator
            ConcatenateContainerWrapper_iterator& operator=(const ConcatenateContainerWrapper_iterator&) noexcept =
                default;
            //! Move Assignment Operator
            ConcatenateContainerWrapper_iterator& operator=(ConcatenateContainerWrapper_iterator&&) noexcept = default;

            //! Pre-Increment
            ConcatenateContainerWrapper_iterator& operator++() const
            {
                forIndex<PreIncrement>();
                return *this;
            }
            //! Post-Increment
            ConcatenateContainerWrapper_iterator operator++(int) const
            {
                ConcatenateContainerWrapper_iterator rv{*this};
                ++*this;
                return rv;
            }
            //! Increment Self
            // clang-format off
            ConcatenateContainerWrapper_iterator& operator+=(difference_type d) const
            requires std::is_same_v<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                throw std::logic_error("Not Implemented");
            }
            //! Addition Operator
            // clang-format off
            ConcatenateContainerWrapper_iterator operator+(difference_type d) const
            requires std::is_same_v<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                throw std::logic_error("Not Implemented");
            }

            //! Pre-Decrement
            ConcatenateContainerWrapper_iterator& operator--() const
            {
                forIndex<PreDecrement>();
                return *this;
            }
            //! Post-Decrement
            ConcatenateContainerWrapper_iterator operator--(int) const
            {
                ConcatenateContainerWrapper_iterator rv{*this};
                --*this;
                return rv;
            }
            //! Decrement Self
            // clang-format off
            ConcatenateContainerWrapper_iterator& operator-=(difference_type d) const
            requires std::is_same_v<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                throw std::logic_error("Not Implemented");
            }
            //! Subtraction Operator
            // clang-format off
            ConcatenateContainerWrapper_iterator operator-(difference_type d) const
            requires std::is_same_v<iterator_category, std::random_access_iterator_tag>
            // clang-format on
            {
                throw std::logic_error("Not Implemented");
            }

           private
               : ConcatenateContainerWrapper_iterator(ConcatenateContainerWrapper<Value, Containers...>* parent,
                                                      std::size_t index,
                                                      auto iterator)
               : m_parent(parent)
               , m_container_index(index)
               , m_internal_iterator(iterator)
            {
                // This could be a concept?
                static_assert(possibleIterator<decltype(iterator), Containers...>());
            }

            // region forIndex
            template <std::size_t N, template <std::size_t> typename Function>
            void forIndex() requires(N == s_num_containers)
            {}

            template <std::size_t N = 0, template <std::size_t> typename Function>
            void forIndex() requires(N < s_num_containers)
            {
                if(m_container_index == N)
                {
                    Function<N> f{this};
                    f();
                }
                else
                {
                    forIndex<N + 1, Function>();
                }
            }
            // endregion
            // region forNextCondition
            // clang-format off
            template <std::size_t N,
                      template <std::size_t> typename Condition,
                      template <std::size_t> typename Function,
                      typename OrFunction>
            // clang-format on
            void forNextCondition() requires(N == s_num_containers)
            {
                OrFunction f{this};
                f();
            }

            // clang-format off
            template <std::size_t N,
                      template <std::size_t> typename Condition,
                      template <std::size_t> typename Function,
                      typename OrFunction>
            // clang-format on
            void forNextCondition() requires(N < s_num_containers)
            {
                Condition<N> condition{this};
                if(condition())
                {
                    Function<N> f{this};
                    f();
                }
                else
                {
                    forNextCondition<N + 1, Condition, Function>();
                }
            }
            // endregion
            // region forPreviousCondition
            // clang-format off
            template <std::size_t N,
                      template <std::size_t> typename Condition,
                      template <std::size_t> typename Function,
                      typename OrFunction>
            // clang-format on
            void forPreviousCondition() requires(N == 0)
            {
                Condition<N> condition{this};
                if(condition())
                {
                    Function<N> f{this};
                    f();
                }
                else
                {
                    OrFunction f{this};
                    f();
                }
            }

            // clang-format off
            template <std::size_t N,
                      template <std::size_t> typename Condition,
                      template <std::size_t> typename Function,
                      typename OrFunction>
            // clang-format on
            void forPreviousCondition() requires(N > 0)
            {
                Condition<N> condition{this};
                if(condition())
                {
                    Function<N> f{this};
                    f();
                }
                else
                {
                    forPreviousCondition<N - 1, Condition, Function>();
                }
            }
            // endregion

            template <std::size_t N>
            struct NonEmptyCondition
            {
                bool operator()()
                {
                    return std::get<N>(parent_iterator->m_parent->m_container)->size() > 0;
                }
                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            template <std::size_t N>
            struct SetBegin
            {
                void operator()()
                {
                    parent_iterator->m_internal_iterator = std::get<N>(parent_iterator->m_parent->m_container)->begin();
                }
                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            struct SetFinalEnd
            {
                void operator()()
                {
                    parent_iterator->m_internal_iterator =
                        std::get<s_last_index>(parent_iterator->m_parent->m_container)->end();
                }
                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            template <std::size_t N>
            struct PreIncrement
            {
                void operator()()
                {
                    auto container_iterator = std::get<N>(parent_iterator->m_internal_iterator);
                    auto container_end      = std::get<N>(parent_iterator->m_parent->m_container)->end();
                    if(container_iterator != container_end)
                    {
                        ++container_iterator;
                        if(container_iterator == container_end)
                        {
                            parent_iterator->forNextCondition<N + 1, NonEmptyCondition, SetBegin, SetFinalEnd>();
                        }
                    }
                    else
                    {
                        throw createLogicError("This shouldn't happen end<N> == begin<N+1>");
                    }
                }

                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            template <std::size_t N>
            struct SetEndMinusOne
            {
                void operator()()
                {
                    parent_iterator->container_iterator =
                        std::get<N>(parent_iterator->m_parent->m_container)->end() - 1;
                }

                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            struct SetFirstBegin
            {
                void operator()()
                {
                    parent_iterator->container_iterator = std::get<0>(parent_iterator->m_parent->m_container)->begin();
                }

                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            template <std::size_t N>
            struct PreDecrement
            {
                void operator()()
                {
                    auto container_iterator = std::get<N>(parent_iterator->m_internal_iterator);
                    auto container_begin    = std::get<N>(parent_iterator->m_parent->m_container)->begin();
                    if(container_iterator != container_begin)
                    {
                        --container_iterator;
                        // We say at the begin
                    }
                    else
                    {
                        parent_iterator
                            ->forPreviousCondition<N - 1, NonEmptyCondition, SetEndMinusOne, SetFirstBegin>();
                    }
                }

                ConcatenateContainerWrapper_iterator<Value, Containers...>* parent_iterator;
            };

            ConcatenateContainerWrapper<Value, Containers...>* m_parent;
            std::size_t m_container_index;
            std::variant<typename Containers::iterator...> m_internal_iterator;

            friend ConcatenateContainerWrapper<Value, Containers...>;
        };
    }  // namespace detail

    /*!
     * \class ConcatenateContainerWrapper
     * \brief A light-weight wrapper that pretends to be 2 or more containers concatenated together
     */
    // clang-format off
    template <typename Value,
              std::ranges::range... Containers>
    // clang-format on
    class ConcatenateContainerWrapper
    {
        static_assert(sizeof...(Containers) > 2);
        constexpr static std::size_t s_num_containers = sizeof...(Containers);
        constexpr static std::size_t s_last_index     = sizeof...(Containers) - 1;

        static_assert(detail::allContainers<Containers...>(),
                      "Containers template parameter pack are not all containers");
        static_assert(detail::IsSameValueContainers<Value, Containers...>(),
                      "Not all of the containers have the same value type");

       public:
        using value_type = Value;
        using size_type  = std::size_t;  //!< This is the standard size_type, so we make an assumption here
        using difference_type =
            std::ptrdiff_t;  //!< This is the standard difference_type, so we make an assumption here
        using reference       = value_type&;
        using const_reference = const value_type&;
        using pointer         = value_type*;
        using const_pointer   = const value_type*;

        using iterator = detail::ConcatenateContainerWrapper_iterator<Value, Containers...>;

        // region Special Member Functions
        // No Default Constructor
        //! Copy Constructor
        ConcatenateContainerWrapper(const ConcatenateContainerWrapper&) = default;
        //! Move Constructor
        ConcatenateContainerWrapper(ConcatenateContainerWrapper&&) noexcept = default;
        //! Destructor
        ~ConcatenateContainerWrapper() = default;
        //! Copy Assignment Operator
        ConcatenateContainerWrapper& operator=(const ConcatenateContainerWrapper&) = default;
        //! Move Assignment Operator
        ConcatenateContainerWrapper& operator=(ConcatenateContainerWrapper&&) noexcept = default;
        // endregion
        ConcatenateContainerWrapper(Containers*... containers)
            : m_containers(containers...)
        {}

        [[nodiscard]] inline reference front()
        {
            return std::get<0>(m_containers)->front();
        }
        [[nodiscard]] inline const_reference front() const
        {
            return std::get<0>(m_containers)->front();
        }
        [[nodiscard]] inline reference back()
        {
            return std::get<s_last_index>(m_containers)->back();
        }
        [[nodiscard]] inline const_reference back() const
        {
            return std::get<s_last_index>(m_containers)->back();
        }

        [[nodiscard]] inline iterator begin()
        {
            return iterator{std::get<0>(m_containers)->begin()};
        }
        //        [[nodiscard]] inline const_iterator begin() const
        //        {
        //            return const_iterator{std::get<0>(m_containers)->begin()};
        //        }
        [[nodiscard]] inline iterator end()
        {
            return iterator{std::get<s_last_index>(m_containers)->end()};
        }
        //        [[nodiscard]] inline const_iterator end() const
        //        {
        //            return const_iterator{std::get<s_last_index>(m_containers)->end()};
        //        }

        [[nodiscard]] inline bool empty() const
        {
            return std::apply(
                [](Containers*... containers)
                {
                    return (containers->empty() & ...);
                },
                m_containers);
        }

        [[nodiscard]] std::size_t size() const
        {
            return std::apply(
                [](Containers*... containers)
                {
                    return (containers->size() + ...);
                },
                m_containers);
        }

       private:
        std::tuple<Containers*...> m_containers;
    };  // class ConcatenateContainerWrapper
}  // namespace grstapse