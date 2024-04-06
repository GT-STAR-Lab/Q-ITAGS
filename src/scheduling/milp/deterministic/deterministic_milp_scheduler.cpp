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
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"

// region Includes
// External
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/view/iota.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/std_extension.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/geometric_planning/environments/ompl_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/common_scheduler_motion_planner_interface.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_name_scheme.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"
#include "grstapse/species.hpp"
// endregion

namespace grstapse
{
    DeterministicMilpScheduler::DeterministicMilpScheduler(
        const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs)
        : DeterministicMilpSchedulerBase(
              problem_inputs,
              std::make_shared<MutexIndicators>(problem_inputs,
                                                std::make_shared<DeterministicMilpSchedulerNameScheme>()),
              std::make_shared<DeterministicMilpSchedulerNameScheme>(),
              std::make_shared<CommonSchedulerMotionPlannerInterface>())
    {}

    std::shared_ptr<const FailureReason> DeterministicMilpScheduler::createObjective(GRBModel& model)
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
        return nullptr;
    }

    UpdateModelResult DeterministicMilpScheduler::updateModel(GRBModel& model)
    {
        UpdateModelResult rv(UpdateModelResultType::e_no_update);

        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        std::vector<int> all_previous_tasks(num_robots, -1);

        std::vector<unsigned int> scheduled_order = m_task_info.scheduledOrder();
        for(unsigned int task_nr: scheduled_order)
        {
            for(const std::shared_ptr<const Robot>& robot: m_problem_inputs->coalition(task_nr))
            {
                auto iter                   = ::ranges::find_if(::ranges::views::iota(0u, num_robots),
                                              [=, this](unsigned int r) -> bool
                                              {
                                                  return robot == m_problem_inputs->robot(r);
                                              });
                const unsigned int robot_nr = *iter;
                const int previous_task_nr  = all_previous_tasks[robot_nr];

                UpdateModelResult update_model_result(UpdateModelResultType::e_no_update);
                // Initial Transition
                if(previous_task_nr == -1)
                {
                    update_model_result = m_task_info.updateTaskLowerBound(task_nr, robot);
                }
                // Other Transitions
                else
                {
                    update_model_result =
                        m_transition_info.updateTransitionDuration(model, previous_task_nr, task_nr, robot);
                }
                switch(update_model_result.type())
                {
                    case UpdateModelResultType::e_no_update:
                    {
                        // Do nothing
                        break;
                    }
                    case UpdateModelResultType::e_updated:
                    {
                        rv = update_model_result;
                        break;
                    }
                    case UpdateModelResultType::e_failure:
                    {
                        return update_model_result;
                    }
                }
                all_previous_tasks[robot_nr] = task_nr;
            }
        }
        return rv;
    }

    std::shared_ptr<const ScheduleBase> DeterministicMilpScheduler::createSchedule(GRBModel& model)
    {
        const double makespan                           = m_makespan.get(GRB_DoubleAttr_X);
        std::vector<std::pair<float, float>> timepoints = m_task_info.timePoints();
        std::vector<std::pair<unsigned int, unsigned int>> precedence_set_mutex_constraints =
            m_mutex_indicators->precedenceSet();
        return std::make_shared<const DeterministicSchedule>(makespan, timepoints, precedence_set_mutex_constraints);
    }

}  // namespace grstapse