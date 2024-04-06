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
#include "grstapse/task_allocation/itags/itags_statistics.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/geometric_planning/motion_planners/motion_planner_base.hpp"
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"

namespace grstapse
{
    ItagsStatistics::ItagsStatistics(const std::string& timer_name)
        : SearchStatisticsCommon(timer_name)
    {}

    nlohmann::json ItagsStatistics::serializeToJson(const std::shared_ptr<const ProblemInputs>& problem_inputs) const
    {
        const auto& itags_problem_inputs = std::dynamic_pointer_cast<const ItagsProblemInputs>(problem_inputs);

        nlohmann::json j = SearchStatisticsCommon::serializeToJson(problem_inputs);

        const float motion_planning_time = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float smp_time             = TimeKeeper::instance().time(constants::k_scheduling_time);
        const float scheduling_time      = smp_time - motion_planning_time;
        const float total_time           = j[constants::k_total_time];
        const float task_allocation_time = total_time - smp_time;

        j[constants::k_task_allocation_time] = task_allocation_time;
        j[constants::k_scheduling_time]      = scheduling_time;
        j[constants::k_motion_planning_time] = motion_planning_time;
        unsigned int num_motion_plans        = 0;
        for(const std::shared_ptr<MotionPlannerBase>& motion_planner: itags_problem_inputs->motionPlanners())
        {
            num_motion_plans += motion_planner->numMotionPlans();
        }
        j[constants::k_num_motion_plans]         = num_motion_plans;
        j[constants::k_num_motion_plan_failures] = MotionPlannerBase::numFailures();
        j[constants::k_num_scheduling_failures]  = SchedulerBase::numFailures();
        j[constants::k_num_scheduling_iterations] =
            MilpSchedulerBase::numIterations() - 1;  //!< The first MILP run is for sched_best

        return j;
    }

}  // namespace grstapse