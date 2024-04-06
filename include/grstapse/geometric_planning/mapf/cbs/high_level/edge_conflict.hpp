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
#include "grstapse/geometric_planning/mapf/cbs/high_level/conflict_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/temporal_edge.hpp"

namespace grstapse
{
    /*!
     * A conflict between two robots' low level plans across an edge
     */
    class EdgeConflict
        : public ConflictBase
        , public TemporalEdge
    {
       public:
        /*!
         * Constructor
         *
         * \param agents
         * \param time
         * \param x1
         * \param y1
         * \param x2
         * \param y2
         */
        EdgeConflict(const std::array<unsigned int, 2>& agents,
                     unsigned int time,
                     unsigned int x1,
                     unsigned int y1,
                     unsigned int x2,
                     unsigned int y2);

        //! \returns Constraints for the two robots that are part of the conflict
        [[nodiscard]] robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>> createConstraints()
            const override;
    };

}  // namespace grstapse