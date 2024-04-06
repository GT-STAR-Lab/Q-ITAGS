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
#include <memory>

// External
#include <robin_hood/robin_hood.hpp>

// Local
#include "grstapse/common/search/a_star/a_star.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    // region Forward Declarations
    class GridCell;
    class GridMap;
    class ParametersBase;
    class TemporalGridCellNode;
    class ConstraintBase;
    // endregion

    /*!
     * \brief An A* search through a temporal grid where cells are <t, x, y>.
     *
     * Commonly used as the low level search for Conflict-Based Search (CBS). A
     * single agent search through a grid where temporospatial constraints have
     * been placed by an external source.
     *
     * \see ConflictBasedSearch
     */
    class SpaceTimeAStarWithConstraints : public AStar<TemporalGridCellNode, SearchStatisticsCommon>
    {
        using Base_ = AStar<TemporalGridCellNode, SearchStatisticsCommon>;

       public:
        // region Special Member Functions
        SpaceTimeAStarWithConstraints()                                         = delete;
        SpaceTimeAStarWithConstraints(const SpaceTimeAStarWithConstraints&)     = delete;
        SpaceTimeAStarWithConstraints(SpaceTimeAStarWithConstraints&&) noexcept = default;
        ~SpaceTimeAStarWithConstraints()                                        = default;
        SpaceTimeAStarWithConstraints& operator=(const SpaceTimeAStarWithConstraints&) = delete;
        SpaceTimeAStarWithConstraints& operator=(SpaceTimeAStarWithConstraints&&) noexcept = default;
        // endregion

        explicit SpaceTimeAStarWithConstraints(
            const std::shared_ptr<const ParametersBase>& parameters,
            const std::shared_ptr<const GridMap>& map,
            const std::shared_ptr<const GridCell>& initial,
            const std::shared_ptr<const GridCell>& goal,
            const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints);

        //! \copydoc BestFirstSearchBase
        [[nodiscard]] std::shared_ptr<TemporalGridCellNode> createRootNode() override final;

       private:
        std::shared_ptr<const GridCell> m_initial;
    };
}  // namespace grstapse