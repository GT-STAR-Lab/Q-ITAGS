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
// Local
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/task_allocation/itags/allocation_percentage_remaining.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/normalized_schedule_quality.hpp"

namespace grstapse
{
    class ItagsProblemInputs;

    namespace detail
    {
        struct TimeExtendedTaskAllocationQualityParameters
        {
            std::shared_ptr<const ItagsProblemInputs> problem_inputs;
            float alpha                                        = 0.25;
            std::shared_ptr<AllocationPercentageRemaining> apr = nullptr;
            std::shared_ptr<NormalizedScheduleQuality> nsq     = nullptr;
        };
    }  // namespace detail

    /*!
     * \brief Computes the Time Extended Task Allocation Quality heuristic
     *
     * This heuristic is a convex combination of Allocation Percentage Remaining and Normalized Schedule Quality
     *
     * \see AllocationPercentageRemaining
     * \see NormalizedScheduleQuality
     * \see Itags
     *
     * \cite Neville, G., Messing, A., Ravichandar, H., Hutchinson, S., & Chernova, S. (2021, August). An interleaved
     *       approach to trait-based task allocation and scheduling. In 2021 IEEE/RSJ International Conference on
     *       Intelligent Robots and Systems (IROS) (pp. 1507-1514). IEEE.
     */
    class TimeExtendedTaskAllocationQuality : public HeuristicBase<IncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        explicit TimeExtendedTaskAllocationQuality(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                                   float alpha                                               = 0.25,
                                                   const std::shared_ptr<AllocationPercentageRemaining>& apr = nullptr,
                                                   const std::shared_ptr<NormalizedScheduleQuality>& nsq     = nullptr);

        //! Constructor
        explicit TimeExtendedTaskAllocationQuality(
            const detail::TimeExtendedTaskAllocationQualityParameters& parameters);

        //! Constructor
        explicit TimeExtendedTaskAllocationQuality(detail::TimeExtendedTaskAllocationQualityParameters&& parameters);

        //! \returns A combination of APR and NSQ heuristics
        [[nodiscard]] float operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const final override;

       protected:
        float m_alpha;
        std::shared_ptr<const AllocationPercentageRemaining> m_apr;
        std::shared_ptr<const NormalizedScheduleQuality> m_nsq;
    };
}  // namespace grstapse