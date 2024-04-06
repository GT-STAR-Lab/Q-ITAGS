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
// External
#include <range/v3/numeric/accumulate.hpp>
// Local
#include "grstapse/common/utilities/std_extension.hpp"
// endregion

namespace grstapse
{
    namespace ranges_ext
    {
        // region min
        namespace detail
        {
            //! A tag to find the correct | operator/overload
            template <typename ReturnType>
            struct MinTag
            {};

            /*!
             * \brief Pipe operator for the min operation
             *
             * \tparam ReturnType
             * \tparam Rng
             *
             * \param r
             * \param
             *
             * \returns
             */
            template <typename ReturnType, std::ranges::input_range Rng>
            requires std::ranges::borrowed_range<Rng>
            constexpr ReturnType operator|(Rng&& rng, MinTag<ReturnType>&&)
            {
                return *(std::ranges::min_element(begin(rng), end(rng)));
            }

            /*!
             * \brief Pipe operator for the min operation
             *
             * \tparam ReturnType
             * \tparam Rng
             *
             * \param r
             * \param
             *
             * \returns
             */
            template <typename ReturnType, std_ext::non_borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MinTag<ReturnType>&&)
            {
                return std::ranges::min(rng);
            }

            //! A tag to find the correct | operator/overload
            template <typename ReturnType>
            struct MinCompTag
            {
                std_ext::binary_function<ReturnType> comp;
            };

            /*!
             * \brief Pipe operator for the min operation
             *
             * \tparam ReturnType
             * \tparam Rng
             *
             * \param r
             * \param
             *
             * \returns
             */
            template <typename ReturnType, std::ranges::borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MinCompTag<ReturnType>&& h)
            {
                return *(std::ranges::min_element(begin(rng), end(rng), std::move(h.comp)));
            }

            /*!
             * \brief Pipe operator for the min operation
             *
             * \tparam ReturnType The return type
             * \tparam Rng
             *
             * \param r The range
             * \param h
             *
             * \returns The maximum element from a range base on std::less
             *
             * \note This version is for non-borrow_ranges, so the output of view operations
             */
            template <typename ReturnType, std_ext::non_borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MinCompTag<ReturnType>&& h)
            {
                return std::ranges::min(rng, std::move(h.comp));
            }
        }  // namespace detail

        /*!
         * \tparam ReturnType
         *
         * \returns The tag for the pipe operator
         */
        template <typename ReturnType>
        auto min()
        {
            return detail::MinTag<ReturnType>{};
        }

        /*!
         * \tparam ReturnType
         *
         * \returns The tag for the pipe operator
         */
        template <typename ReturnType>
        auto min(const std_ext::binary_function<ReturnType>& comp)
        {
            return detail::MinCompTag<ReturnType>{.comp = comp};
        }

        /*!
         * \tparam ReturnType
         *
         * \returns The tag for the pipe operator
         */
        template <typename ReturnType>
        auto min(std_ext::binary_function<ReturnType>&& comp)
        {
            return detail::MinCompTag<ReturnType>{.comp = std::move(comp)};
        }
        // endregion
        // region max
        namespace detail
        {
            //! A tag to find the correct | operator/overload
            template <typename ReturnType>
            struct MaxTag
            {};

            /*!
             * \brief Pipe operator for the max operation
             *
             * \tparam ReturnType The return type
             * \tparam Rng The range type
             *
             * \returns The maximum element from a range base on std::less
             *
             * \note This version is for borrow_ranges (which includes containers)
             */
            template <typename ReturnType, std::ranges::borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MaxTag<ReturnType>&&)
            {
                return *(std::ranges::max_element(begin(rng), end(rng)));
            }

            /*!
             * \brief Pipe operator for the max operation
             *
             * \tparam ReturnType The return type
             * \tparam R The range type
             *
             * \returns The maximum element from a range base on std::less
             *
             * \note This version is for non-borrow_ranges, so the output of view operations
             */
            template <typename ReturnType, std_ext::non_borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MaxTag<ReturnType>)
            {
                return std::ranges::max(rng);
            }

            //! A tag to find the correct | operator/overload
            template <typename ReturnType>
            struct MaxCompTag
            {
                std_ext::binary_function<ReturnType> comp;
            };

            template <typename ReturnType, std::ranges::borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MaxCompTag<ReturnType>&& h)
            {
                return *(std::ranges::max_element(begin(rng), end(rng), std::move(h.comp)));
            }

            template <typename ReturnType, std_ext::non_borrowed_range Rng>
            constexpr ReturnType operator|(Rng&& rng, MaxCompTag<ReturnType>&& h)
            {
                return std::ranges::max(rng, std::move(h.comp));
            }
        }  // namespace detail

        template <typename ReturnType>
        auto max()
        {
            return detail::MaxTag<ReturnType>{};
        }

        template <typename ReturnType>
        auto max(const std_ext::binary_function<ReturnType>& comp)
        {
            return detail::MaxCompTag<ReturnType>{.comp = comp};
        }

        template <typename ReturnType>
        auto max(std_ext::binary_function<ReturnType>&& comp)
        {
            return detail::MaxCompTag<ReturnType>{.comp = std::move(comp)};
        }
        // endregion
        // region accumulate
        namespace detail
        {
            //! A tag to find the correct | operator/overload
            template <typename ReturnType>
            struct AccumulateTag
            {
                ReturnType init;
                std_ext::binary_function<ReturnType> binary_operation;
            };

            template <typename ReturnType, std::ranges::input_range R>
            constexpr ReturnType operator|(R&& rng, AccumulateTag<ReturnType>&& h)
            {
                return std::accumulate(begin(rng), end(rng), std::move(h.init), std::move(h.binary_operation));
            }
        }  // namespace detail

        template <typename ReturnType>
        auto accumulate(ReturnType init, const std_ext::binary_function<ReturnType>& binary_operation)
        {
            return detail::AccumulateTag{.init = std::move(init), .binary_operation = binary_operation};
        }

        template <typename ReturnType>
        auto accumulate(ReturnType init, std_ext::binary_function<ReturnType>&& binary_operation)
        {
            return detail::AccumulateTag{.init = std::move(init), .binary_operation = std::move(binary_operation)};
        }
        // endregion
        // region sum
        template <typename ReturnType>
        auto sum(ReturnType init = ReturnType(0))
        {
            return detail::AccumulateTag<ReturnType>{
                .init             = std::move(init),
                .binary_operation = [](const ReturnType& lhs, const ReturnType& rhs) -> ReturnType
                {
                    return lhs + rhs;
                }};
        }
        // endregion
        // region product
        template <typename ReturnType>
        auto product(ReturnType init = ReturnType(1))
        {
            return detail::AccumulateTag{
                .init             = std::move(init),
                .binary_operation = [](const ReturnType& lhs, const ReturnType& rhs) -> ReturnType
                {
                    return lhs * rhs;
                }};
        }
        // endregion
        // region find
        namespace detail
        {
            template <typename ReturnType>
            struct FindTag
            {
                ReturnType value;
            };

            template <typename ReturnType, std::ranges::input_range Rng>
            ReturnType operator|(Rng&& rng, FindTag<ReturnType>&& h)
            {
                return *std::find(begin(rng), end(rng), std::move(h.value));
            }
        }  // namespace detail

        template <typename ReturnType>
        auto find(ReturnType value)
        {
            return detail::FindTag<ReturnType>{.value = std::move(value)};
        }
        // endregion
        // region find_if
        namespace detail
        {
            template <typename ReturnType>
            struct FindIfTag
            {
                std_ext::unary_predicate<ReturnType> up;
            };

            template <typename ReturnType, std::ranges::input_range R>
            ReturnType operator|(R&& rng, FindIfTag<ReturnType>&& h)
            {
                return *std::find_if(begin(rng), end(rng), std::move(h.up));
            }
        }  // namespace detail

        template <typename ReturnType>
        auto find_if(const std_ext::unary_predicate<ReturnType>& up)
        {
            return detail::FindIfTag<ReturnType>{.up = up};
        }

        template <typename ReturnType>
        auto find_if(std_ext::unary_predicate<ReturnType>&& up)
        {
            return detail::FindIfTag<ReturnType>{.up = std::move(up)};
        }
        // endregion
    }  // namespace ranges_ext
}  // namespace grstapse