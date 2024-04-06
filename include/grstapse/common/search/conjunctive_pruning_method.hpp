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

// Local
#include "grstapse/common/search/multi_pruning_method_base.hpp"

namespace grstapse
{
    /*!
     * \brief Pruning Method that is a conjunction of multiple pruning methods
     *
     * \tparam SearchNodeDeriv A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    class ConjunctivePruningMethod : public MultiPruningMethodBase<SearchNode>
    {
        using Base_ = MultiPruningMethodBase<SearchNode>;

       public:
        //! Constructor
        explicit ConjunctivePruningMethod(
            const std::vector<std::shared_ptr<PruningMethodBase<SearchNode>>>& methods = {})
            : Base_(methods)
        {}

        //! Constructor
        explicit ConjunctivePruningMethod(std::vector<std::shared_ptr<const PruningMethodBase<SearchNode>>>&& methods)
            : Base_(std::move(methods))
        {}

        //! \returns Whether to prune this node from the search
        [[nodiscard]] virtual bool operator()(const std::shared_ptr<const SearchNode>& node) const final override
        {
            for(const std::shared_ptr<const PruningMethodBase<SearchNode>>& method: Base_::m_submethods)
            {
                if(!method->operator()(node))
                {
                    return false;
                }
            }
            return true;
        }
    };

}  // namespace grstapse