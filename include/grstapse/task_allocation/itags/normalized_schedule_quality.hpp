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
#include <set>
#include <tuple>
// Local
#include "grstapse/common/search/heuristic_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"
#include "grstapse/scheduling/schedule_base.hpp"
#include "grstapse/scheduling/scheduler_base.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    /*!
     * Evaluates an allocation based on the quality of the makespan from the associated schedule
     *
     * \see Itags
     *
     * \cite Neville, G., Messing, A., Ravichandar, H., Hutchinson, S., & Chernova, S. (2021, August). An interleaved
     *       approach to trait-based task allocation and scheduling. In 2021 IEEE/RSJ International Conference on
     *       Intelligent Robots and Systems (IROS) (pp. 1507-1514). IEEE.
     */
    class NormalizedScheduleQuality : public HeuristicBase<IncrementalTaskAllocationNode>
    {
       public:
        //! \brief Constructor
        explicit NormalizedScheduleQuality(
            const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
            const std::function<void(const std::shared_ptr<const SchedulerResult>&)>& on_failure =
                [](const std::shared_ptr<const SchedulerResult>&) {},
            std::function<void(const std::shared_ptr<const SchedulerResult>&)> on_success =
                [](const std::shared_ptr<const SchedulerResult>&) {});

        //! \brief Constructor
        explicit NormalizedScheduleQuality(
            const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
            const std::function<std::shared_ptr<SchedulerBase>(const std::shared_ptr<const SchedulerProblemInputs>&)>&
                create_scheduler,
            const std::function<void(const std::shared_ptr<const SchedulerResult>&)>& on_failure =
                [](const std::shared_ptr<const SchedulerResult>&) {},
            std::function<void(const std::shared_ptr<const SchedulerResult>&)> on_success =
                [](const std::shared_ptr<const SchedulerResult>&) {});

        //! \returns The quality of the makespan of the associated schedule
        [[nodiscard]] float operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const final override;

        //! \returns The quality of the makespan of the associated schedule
        [[nodiscard]] virtual float operator()(IncrementalTaskAllocationNode* node) const;

       protected:
        //! \returns The makespan for the associated schedule of \p node
        [[nodiscard]] virtual float computeMakespan(IncrementalTaskAllocationNode* node) const;

        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        std::function<std::shared_ptr<SchedulerBase>(const std::shared_ptr<const SchedulerProblemInputs>&)>
            m_create_scheduler;
        std::function<void(const std::shared_ptr<const SchedulerResult>&)> m_on_failure;
        std::function<void(const std::shared_ptr<const SchedulerResult>&)> m_on_success;
    };
}  // namespace grstapse