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

// Local
#include "grstapse/common/search/path_cost_base.hpp"
#include "grstapse/geometric_planning/grid/grid_cell.hpp"

namespace grstapse
{
    /*!
     * Calculates the cost of the path to a temporal grid cell based on
     * its ancestry
     *
     * \tparam GridCellDerive A derivative class of GridCell (Also implicitly a derivative of NodeBase
     *                        through the PathCostBase)
     */
    template <typename GridCellDerive>
    requires std::derived_from<GridCellDerive, GridCell>
    // Note: GridCellDerive deriving from SearchNodeBase is checked in PathCostBase
    class GridCellPathCost : public PathCostBase<GridCellDerive>
    {
       public:
        //! \copydoc PathCostBase
        [[nodiscard]] float operator()(const std::shared_ptr<GridCellDerive>& child) const final override
        {
            std::shared_ptr<const GridCellDerive> parent = child->parent();
            if(parent != nullptr)
            {
                return parent->g() + parent->euclideanDistance(*child);
            }
            return 0.0;
        }
    };
}  // namespace grstapse