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
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_subscheduler.hpp"

// External
#include <range/v3/all.hpp>
// Local
#include "grstapse/common/milp/milp_utilties.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/std_extension.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/subscheduler_motion_planner_interface.hpp"
#include "grstapse/scheduling/milp/deterministic/subscheduler_name_scheme.hpp"

namespace grstapse
{
    DeterministicMilpSubscheduler::DeterministicMilpSubscheduler(
        unsigned int index,
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<MutexIndicators>& mutex_indicators,
        bool master)
        : DeterministicMilpSchedulerBase(problem_inputs,
                                         mutex_indicators,
                                         std::make_shared<SubschedulerNameScheme>(index),
                                         std::make_shared<SubschedulerMotionPlannerInterface>(index))
        , m_index(index)
        , m_master(master)
    {}

    std::shared_ptr<const FailureReason> DeterministicMilpSubscheduler::createObjective(GRBModel& model)
    {
        if(!m_master)
        {
            // Set all optimization to minimize (is the default, but we explicitly set anyway)
            model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            if(m_problem_inputs->schedulerParameters()->contains(constants::k_use_hierarchical_objective) &&
               m_problem_inputs->schedulerParameters()->get<bool>(constants::k_use_hierarchical_objective))
            {
                // Note: lower priority objectives cannot degrade higher priority objectives
                model.setObjectiveN(GRBLinExpr(m_makespan), 0, 1);  //!< objective, index, priority

                // Bottom level of the object is starting each task as soon as possible (so minimizing start time)
                const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
                for(unsigned int i = 0; i < num_tasks; ++i)
                {
                    model.setObjectiveN(GRBLinExpr(m_task_info.taskStartTimePointVariable(i)),
                                        i + 1,
                                        0);  //!< objective, index, priority
                }
            }
            else
            {
                model.setObjective(GRBLinExpr(m_makespan));
            }
        }
        return nullptr;
    }

    std::shared_ptr<const ScheduleBase> DeterministicMilpSubscheduler::createSchedule(GRBModel& model)
    {
        // Note: Intentional
        throw std::logic_error("Not implemented");
    }

    double DeterministicMilpSubscheduler::longestFixedChain() const
    {
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
        std::set<unsigned int> has_predecessor =
            m_problem_inputs->precedenceConstraints() | ranges::views::values | ranges::to<std::set<unsigned int>>();
        std::vector<float> distance = ranges::views::iota(0u, num_tasks) |
                                      ranges::views::transform(
                                          [this, &has_predecessor](unsigned int i) -> float
                                          {
                                              return has_predecessor.contains(i)
                                                         ? std::numeric_limits<float>::infinity()
                                                         : m_task_info.taskLowerBound(i);
                                          }) |
                                      ranges::to<std::vector<float>>();

        // Based on bellman ford
        for(unsigned int v = 0; v < num_tasks - 1; ++v)
        {
            for(auto [i, j]: m_problem_inputs->precedenceConstraints())
            {
                const float task_duration       = m_task_info.taskDuration(i);
                const float transition_duration = m_transition_info.transitionDurationLowerBound(i, j);
                const float j_lowerbound        = m_task_info.taskLowerBound(j);

                const float weight = distance[i] + task_duration + transition_duration;
                if(weight < distance[j] && weight > j_lowerbound)
                {
                    distance[j] = weight;
                }
            }
        }

        // Any left infinite means the initial transition is the lowerbound
        for(unsigned int i = 0; i < num_tasks; ++i)
        {
            if(distance[i] == std::numeric_limits<float>::infinity())
            {
                distance[i] = m_task_info.taskLowerBound(i);
            }
        }

        return ranges::max(distance);
    }

    double DeterministicMilpSubscheduler::dualCutAlphaComponent(GRBModel& model) const
    {
        double rv = 0.0;
        for(unsigned int task_nr = 0, num_tasks = m_problem_inputs->numberOfPlanTasks(); task_nr < num_tasks; ++task_nr)
        {
            const double alpha = constraintDualValue(model, m_name_scheme->createMakespanConstraintName(task_nr));
            rv += m_task_info.taskDuration(task_nr) * alpha;
        }
        return rv;
    }
}  // namespace grstapse