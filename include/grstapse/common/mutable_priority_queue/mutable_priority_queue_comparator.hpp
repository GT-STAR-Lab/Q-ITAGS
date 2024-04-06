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
#include "grstapse/common/mutable_priority_queue/mutable_priority_queueable.hpp"

namespace grstapse
{
    /*!
     * \brief Comparator of MutablePriorityQueueable objects
     *
     * \tparam PriorityType
     * \tparam PayloadType
     *
     * \note The boost priority queue which is under the hood
     *       of MutablePriorityQueue is a max heap, so we use
     *       greater than functions to turn it into a min heap
     */
    template <typename PriorityType, typename PayloadType>
    requires std::derived_from<PayloadType, MutablePriorityQueueable<PriorityType>>
    struct MutablePriorityQueueComparator
    {
        //! \returns Whether \p lhs has a higher priority than \p rhs
        [[nodiscard]] bool operator()(const PayloadType& lhs, const PayloadType& rhs) const
        {
            return lhs.priority() > rhs.priority();
        }

        //! \returns Whether \p lhs has a higher priority than \p rhs
        [[nodiscard]] bool operator()(std::shared_ptr<const PayloadType> lhs,
                                      std::shared_ptr<const PayloadType> rhs) const
        {
            return lhs->priority() > rhs->priority();
        }
    };

}  // namespace grstapse