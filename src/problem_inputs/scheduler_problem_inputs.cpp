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
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"

// External
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    SchedulerProblemInputs::SchedulerProblemInputs(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                                   const Eigen::MatrixXf& allocation)
        : m_itags_problem_inputs(problem_inputs)
        , m_mutex_constraints(computeMutexConstraints(allocation))
        , m_allocation(allocation)
    {}

    SchedulerProblemInputs::SchedulerProblemInputs(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                                   Eigen::MatrixXf&& allocation)
        : m_itags_problem_inputs(problem_inputs)
        , m_mutex_constraints(computeMutexConstraints(allocation))
        , m_allocation(std::move(allocation))
    {}

    void SchedulerProblemInputs::validate() const
    {
        unsigned int num_plan_task = numberOfPlanTasks();
        for(const std::pair<unsigned int, unsigned int>& constraint: m_mutex_constraints)
        {
            if(constraint.first >= num_plan_task || constraint.second >= num_plan_task)
            {
                throw createLogicError("Precedence constraint out of range of the number of plan tasks");
            }
        }
        m_itags_problem_inputs->validate();
    }

    CoalitionView SchedulerProblemInputs::coalition(unsigned int task_nr) const
    {
        return ::ranges::views::iota(0u, numberOfRobots()) |
               ::ranges::views::filter(std::function(
                   [this, task_nr](unsigned int r) -> bool
                   {
                       return m_allocation(task_nr, r) > 0.5f;
                   })) |
               ::ranges::views::transform(std::function(
                   [this](unsigned int r) -> const std::shared_ptr<const Robot>&
                   {
                       return robot(r);
                   }));
    }
    CoalitionView SchedulerProblemInputs::transitionCoalition(unsigned int i, unsigned int j) const
    {
        return ::ranges::views::iota(0u, numberOfRobots()) |
               ::ranges::views::filter(std::function(
                   [this, i, j](unsigned int r) -> bool
                   {
                       return m_allocation(i, r) > 0.5f && m_allocation(j, r);
                   })) |
               ::ranges::views::transform(std::function(
                   [this](unsigned int r) -> const std::shared_ptr<const Robot>&
                   {
                       return robot(r);
                   }));
    }
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<grstapse::SchedulerProblemInputs>
    adl_serializer<std::shared_ptr<grstapse::SchedulerProblemInputs>>::from_json(const nlohmann::json& j)
    {
        auto itags_problem_inputs = j.get<std::shared_ptr<grstapse::ItagsProblemInputs>>();
        auto allocation           = j[grstapse::constants::k_allocation].get<Eigen::MatrixXf>();
        return std::make_shared<grstapse::SchedulerProblemInputs>(itags_problem_inputs, std::move(allocation));
    }
}  // namespace nlohmann