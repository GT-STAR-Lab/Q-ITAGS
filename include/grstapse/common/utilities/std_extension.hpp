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
#include <algorithm>
#include <concepts>
#include <functional>
#include <numeric>
#include <type_traits>
// endregion

/*!
 * \file std_extension.hpp
 *
 * \todo flatten tuples/pairs
 */

namespace grstapse
{
    namespace std_ext
    {
        // region Concepts
        // region hashable
        /*!
         * Declaration of the concept "Hashable", which is satisfied by any type 'T'
         * such that for values 'a' of type 'T', the expression std::hash<T>{}(a)
         * compiles and its result is convertible to std::size_t
         */
        template <typename T>
        concept Hashable = requires(T a)
        {
            {
                std::hash<T>{}(a)
                } -> std::convertible_to<std::size_t>;
        };
        // endregion
        // region comparables
        namespace detail
        {
            template <typename T, typename U>
            concept WeaklLtCmpWith = requires(std::__detail::__cref<T> t, std::__detail::__cref<U> u)
            {
                {
                    t < u
                    } -> std::__detail::__boolean_testable;
                {
                    u < t
                    } -> std::__detail::__boolean_testable;
            };
        }  // namespace detail

        /*!
         * Declaration of the concept "LessThanComparable", which is satisfied by any type 'T'
         * such that for values 'a' and 'b' of type 'T', the expression a < b
         * compiles and its result is convertible to bool
         */
        template <typename T>
        concept LessThanComparable = detail::WeaklLtCmpWith<T, T>;

        namespace detail
        {
            template <typename T, typename U>
            concept WeaklyGtCmpWith = requires(std::__detail::__cref<T> t, std::__detail::__cref<U> u)
            {
                {
                    t > u
                    } -> std::__detail::__boolean_testable;
                {
                    u > t
                    } -> std::__detail::__boolean_testable;
            };
        }  // namespace detail

        /*!
         * Declaration of the concept "GreaterThanComparable", which is satisfied by any type 'T'
         * such that for values 'a' and 'b' of type 'T', the expression a > b
         * compiles and its result is convertible to bool
         */
        template <typename T>
        concept GreaterThanComparable = detail::WeaklyGtCmpWith<T, T>;

        namespace detail
        {
            template <typename T, typename U>
            concept WeaklyLteCmpWith = requires(std::__detail::__cref<T> t, std::__detail::__cref<U> u)
            {
                {
                    t <= u
                    } -> std::__detail::__boolean_testable;
                {
                    u <= t
                    } -> std::__detail::__boolean_testable;
            };
        }  // namespace detail

        /*!
         * Declaration of the concept "LessThanEqualComparable", which is satisfied by any type 'T'
         * such that for values 'a' and 'b' of type 'T', the expression a <= b
         * compiles and its result is convertible to bool
         */
        template <typename T>
        concept LessThanEqualComparable = detail::WeaklyLteCmpWith<T, T>;

        namespace detail
        {
            template <typename T, typename U>
            concept WeaklyGteCmpWith = requires(std::__detail::__cref<T> t, std::__detail::__cref<U> u)
            {
                {
                    t >= u
                    } -> std::__detail::__boolean_testable;
                {
                    u >= t
                    } -> std::__detail::__boolean_testable;
            };
        }  // namespace detail

        /*!
         * Declaration of the concept "GreaterThanEqualComparable", which is satisfied by any type 'T'
         * such that for values 'a' and 'b' of type 'T', the expression a >= b
         * compiles and its result is convertible to bool
         */
        template <typename T>
        concept GreaterThanEqualComparable = detail::WeaklyGteCmpWith<T, T>;

        namespace detail
        {
            template <typename T, typename U>
            concept WeaklyCmpWith = requires(std::__detail::__cref<T> t, std::__detail::__cref<U> u)
            {
                {
                    t < u
                    } -> std::__detail::__boolean_testable;
                {
                    t > u
                    } -> std::__detail::__boolean_testable;
                {
                    t <= u
                    } -> std::__detail::__boolean_testable;
                {
                    t >= u
                    } -> std::__detail::__boolean_testable;
                {
                    t == u
                    } -> std::__detail::__boolean_testable;
                {
                    t != u
                    } -> std::__detail::__boolean_testable;
                {
                    u < t
                    } -> std::__detail::__boolean_testable;
                {
                    u > t
                    } -> std::__detail::__boolean_testable;
                {
                    u <= t
                    } -> std::__detail::__boolean_testable;
                {
                    u >= t
                    } -> std::__detail::__boolean_testable;
                {
                    u == t
                    } -> std::__detail::__boolean_testable;
                {
                    u != t
                    } -> std::__detail::__boolean_testable;
            };
        }  // namespace detail

        /*!
         * Declaration of the concept "Comparable", which is satisfied by any type 'T'
         * that contains all binary operators (<, <=, >, >=, ==, !=)
         */
        template <typename T>
        concept Comparable = detail::WeaklyCmpWith<T, T>;
        // endregion
        // region enum
        /*!
         * Declaration of the concept "Enum", which is satisfied by any type 'T'
         * that is an enum
         */
        template <typename T>
        concept Enum = std::is_enum_v<T>;
        // endregion
        // region non_borrowed_range
        /*!
         * \brief Opposite of borrowed_range
         *
         * \see https://en.cppreference.com/w/cpp/ranges/borrowed_range
         */
        template <typename R>
        concept non_borrowed_range = std::ranges::range<R> &&
            (!std::is_lvalue_reference_v<R> && !std::ranges::enable_borrowed_range<std::remove_cvref_t<R>>);
        // endregion
        // region Container
        template <class T>
        concept Container = requires(T a, const T b)
        {
            requires std::regular<T>;
            requires std::swappable<T>;
            requires std::destructible<typename T::value_type>;
            requires std::same_as<typename T::reference, typename T::value_type&>;
            requires std::same_as<typename T::const_reference, const typename T::value_type&>;
            requires std::forward_iterator<typename T::iterator>;
            requires std::forward_iterator<typename T::const_iterator>;
            requires std::signed_integral<typename T::difference_type>;
            requires std::same_as<typename T::difference_type,
                                  typename std::iterator_traits<typename T::iterator>::difference_type>;
            requires std::same_as<typename T::difference_type,
                                  typename std::iterator_traits<typename T::const_iterator>::difference_type>;
            // clang-format off
            { a.begin() } -> std::same_as<typename T::iterator>;
            { a.end() } -> std::same_as<typename T::iterator>;
            { b.begin() } -> std::same_as<typename T::const_iterator>;
            { b.end() } -> std::same_as<typename T::const_iterator>;
            { a.cbegin() } -> std::same_as<typename T::const_iterator>;
            { a.cend() } -> std::same_as<typename T::const_iterator>;
            { a.size() } -> std::same_as<typename T::size_type>;
            { a.max_size() } -> std::same_as<typename T::size_type>;
            { a.empty() } -> std::same_as<bool>;
            // clang-format on
        };
        // endregion
        // region HasFrontBack
        template <typename T>
        concept HasFrontBack = requires(T a, const T b)
        {
            // clang-format off
            {a.front()} -> std::same_as<typename T::reference>;
            {b.front()} -> std::same_as<typename T::const_reference>;
            {a.back()} -> std::same_as<typename T::reference>;
            {b.back()} -> std::same_as<typename T::const_reference>;
            // clang-format on
        };
        // endregion
        // endregion
        // region Function typdefs
        template <typename ReturnType, typename ArgumentType>
        using unary_function = std::function<ReturnType(ArgumentType)>;
        template <typename ArgumentType>
        using unary_predicate = std::function<bool(ArgumentType)>;
        template <typename ReturnType, typename ArgumentType1 = ReturnType, typename ArgumentType2 = ArgumentType1>
        using binary_function = std::function<ReturnType(ArgumentType1, ArgumentType2)>;
        // endregion
    }  // namespace std_ext
}  // namespace grstapse