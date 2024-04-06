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
#include "grstapse/common/search/pruning_method_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    // Forward Declarations
    class ConstraintBase;
    class EdgeConstraint;
    class VertexConstraint;

    /*!
     * Prunes TemporalGridCells based on a set of constraints
     */
    class PruneConstraints : public PruningMethodBase<TemporalGridCellNode>
    {
       public:
        /*!
         *
         * \param constraints
         */
        explicit PruneConstraints(const robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>>& constraints);

        /*!
         * \returns Whether the node should be pruned
         */
        [[nodiscard]] bool operator()(const std::shared_ptr<const TemporalGridCellNode>& node) const override;

       private:
        /*!
         * \returns Whether this node violates the edge constraint
         */
        [[nodiscard]] bool prune(const std::shared_ptr<const TemporalGridCellNode>& node,
                                 const std::shared_ptr<const EdgeConstraint>& constraint) const;

        /*!
         * \returns Whether this node violates the vertex constraint
         */
        [[nodiscard]] bool prune(const std::shared_ptr<const TemporalGridCellNode>& node,
                                 const std::shared_ptr<const VertexConstraint>& constraint) const;

        robin_hood::unordered_set<std::shared_ptr<const ConstraintBase>> m_constraints;
    };
}  // namespace grstapse