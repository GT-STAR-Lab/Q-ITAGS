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
#include <utility>

// External
#include <boost/functional/hash.hpp>

// Local
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/temporal_edge.hpp"

namespace grstapse
{
    //! \brief A constraint for a specifc agent to not traverse a specific edge at a specific time
    class EdgeConstraint
        : public ConstraintBase
        , public TemporalEdge
    {
       public:
        //! Constructor
        EdgeConstraint(unsigned int time, unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

        using TemporalEdge::operator==;
        using TemporalEdge::operator!=;

        //! \copydoc ConstraintBase
        [[nodiscard]] size_t hash() const override;
    };
}  // namespace grstapse