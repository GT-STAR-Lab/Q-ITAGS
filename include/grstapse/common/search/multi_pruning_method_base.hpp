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
#include "grstapse/common/search/pruning_method_base.hpp"

namespace grstapse
{
    /*!
     * \brief Base class for pruning methods with multiple sub-pruning methods
     *
     * \tparam SearchNodeDeriv
     */
    template <SearchNodeDeriv SearchNode>
    class MultiPruningMethodBase : public PruningMethodBase<SearchNode>
    {
       public:
        //! Adds a sub-method
        inline void add(const std::shared_ptr<PruningMethodBase<SearchNode>>& method)
        {
            m_submethods.push_back(method);
        }

        //! \returns A list of the sub-pruning methods
        [[nodiscard]] inline const std::vector<std::shared_ptr<PruningMethodBase<SearchNode>>>& submethods() const
        {
            return m_submethods;
        }

       protected:
        //! Constructor
        explicit MultiPruningMethodBase(const std::vector<std::shared_ptr<PruningMethodBase<SearchNode>>>& methods = {})
            : m_submethods(methods)
        {}

        //! Constructor
        explicit MultiPruningMethodBase(std::vector<std::shared_ptr<PruningMethodBase<SearchNode>>>&& methods)
            : m_submethods{std::move(methods)}
        {}

        std::vector<std::shared_ptr<PruningMethodBase<SearchNode>>> m_submethods;
    };

}  // namespace grstapse