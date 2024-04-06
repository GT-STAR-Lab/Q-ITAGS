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
#include <map>
// External
#include <magic_enum/magic_enum.hpp>
#include <robin_hood/robin_hood.hpp>
// Local
#include "grstapse/common/utilities/std_extension.hpp"

/*!
 * \brief These are extensions to the magic_enum library
 *
 * We keep them in our namespace to not mess up theirs. See extern/magic_enum for the original
 * code and their license
 *
 * \note We are using their function name scheme for consistency
 *
 * \file magic_enum_extensions.hpp
 */

namespace grstapse::magic_enum_ext
{
    namespace detail
    {
        template <std_ext::Enum E, std::size_t... I>
        [[nodiscard]] auto enum_map_entries(std::index_sequence<I...>) noexcept
        {
            return std::map<std::string_view, E>{
                {magic_enum::detail::enum_name_v<E, magic_enum::detail::values_v<E>[I]>,
                 magic_enum::detail::values_v<E>[I]}...};
        }

        template <std_ext::Enum E, std::size_t... I>
        [[nodiscard]] auto enum_map_entries_string(std::index_sequence<I...>) noexcept
        {
            return std::map<std::string, E>{
                {std::string(magic_enum::detail::enum_name_v<E, magic_enum::detail::values_v<E>[I]>),
                 magic_enum::detail::values_v<E>[I]}...};
        }

        template <std_ext::Enum E, std::size_t... I>
        [[nodiscard]] auto enum_unordered_map_entries(std::index_sequence<I...>) noexcept
        {
            return std::unordered_map<std::string_view, E>{
                {magic_enum::detail::enum_name_v<E, magic_enum::detail::values_v<E>[I]>,
                 magic_enum::detail::values_v<E>[I]}...};
        }

        template <std_ext::Enum E, std::size_t... I>
        [[nodiscard]] auto enum_unordered_map_entries_string(std::index_sequence<I...>) noexcept
        {
            return std::unordered_map<std::string, E>{
                {std::string(magic_enum::detail::enum_name_v<E, magic_enum::detail::values_v<E>[I]>),
                 magic_enum::detail::values_v<E>[I]}...};
        }

    }  // namespace detail

    /*!
     * \tparam E An enum type
     *
     * \returns A map of [string, enumerator] for E
     */
    template <std_ext::Enum E>
    [[nodiscard]] std::map<std::string_view, E> enum_map_entries() noexcept
    {
        return detail::enum_map_entries<E>(std::make_index_sequence<magic_enum::detail::count_v<E>>{});
    }

    /*!
     * \tparam E An enum type
     *
     * \returns A map of [string, enumerator] for E
     */
    template <std_ext::Enum E>
    [[nodiscard]] std::map<std::string, E> enum_map_entries_string() noexcept
    {
        return detail::enum_map_entries_string<E>(std::make_index_sequence<magic_enum::detail::count_v<E>>{});
    }

    /*!
     * \tparam E An enum type
     *
     * \returns An unordered_map of [string, enumerator] for E
     */
    template <std_ext::Enum E>
    [[nodiscard]] std::unordered_map<std::string_view, E> enum_unordered_map_entries() noexcept
    {
        return detail::enum_unordered_map_entries<E>(std::make_index_sequence<magic_enum::detail::count_v<E>>{});
    }

    /*!
     * \tparam E An enum type
     *
     * \returns An unordered_map of [string, enumerator] for E
     */
    template <std_ext::Enum E>
    [[nodiscard]] std::unordered_map<std::string, E> enum_unordered_map_entries_string() noexcept
    {
        return detail::enum_unordered_map_entries_string<E>(std::make_index_sequence<magic_enum::detail::count_v<E>>{});
    }

}  // namespace grstapse::magic_enum_ext