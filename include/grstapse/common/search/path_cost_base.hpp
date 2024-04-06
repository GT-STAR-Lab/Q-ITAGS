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
#include <concepts>
#include <memory>

// Local
#include "grstapse/common/search/search_node_base.hpp"
#include "grstapse/common/utilities/noncopyable.hpp"

namespace grstapse
{
    /*!
     * \brief An interface for defining how to compute the path cost for a node during search
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    class PathCostBase : private Noncopyable
    {
       public:
        //! \returns The path cost of a node in graph/tree
        [[nodiscard]] virtual float operator()(const std::shared_ptr<SearchNode>& node) const = 0;

       protected:
        PathCostBase() = default;
    };

    /*!
     * \brief
     *
     * \tparam T
     * \tparam SearchNode
     */
    template <typename T, typename SearchNode>
    concept PathCostDeriv = std::derived_from<T, PathCostBase<SearchNode>>;
}  // namespace grstapse