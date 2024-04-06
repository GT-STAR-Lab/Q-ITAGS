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
#include "grstapse/scheduling/milp/deterministic/subscheduler_motion_planner_interface.hpp"

// Local
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner_base.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/task.hpp"

namespace grstapse
{
    SubschedulerMotionPlannerInterface::SubschedulerMotionPlannerInterface(unsigned int index)
        : m_index(index)
    {}

    float SubschedulerMotionPlannerInterface::computeTaskDuration(
        const std::shared_ptr<const Task>& task,
        const std::vector<std::shared_ptr<const Robot>>& coalition) const
    {
        if(coalition.empty())
        {
            return task->staticDuration();
        }

        std::shared_ptr<const Robot> widest_robot = nullptr;
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            if(widest_robot == nullptr || robot->boundingRadius() > widest_robot->boundingRadius())
            {
                widest_robot = robot;
            }
        }
        auto motion_planner =
            std::dynamic_pointer_cast<SampledEuclideanGraphMotionPlannerBase>(widest_robot->species()->motionPlanner());
        return motion_planner->durationQuery(
                   m_index,
                   widest_robot->species(),
                   std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(task->initialConfiguration()),
                   std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(task->terminalConfiguration())) +
               task->staticDuration();
    }

    bool SubschedulerMotionPlannerInterface::isInitialTransitionMemoized(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledEuclideanGraphMotionPlannerBase>(robot->species()->motionPlanner());
        return motion_planner->isMemoized(
            m_index,
            robot->species(),
            std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(robot->initialConfiguration()),
            std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(configuration));
    }

    float SubschedulerMotionPlannerInterface::computeInitialTransitionDuration(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledEuclideanGraphMotionPlannerBase>(robot->species()->motionPlanner());
        return motion_planner->durationQuery(
            m_index,
            robot->species(),
            std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(robot->initialConfiguration()),
            std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(configuration));
    }

    bool SubschedulerMotionPlannerInterface::isTransitionMemoized(
        const std::shared_ptr<const ConfigurationBase>& initial,
        const std::shared_ptr<const ConfigurationBase>& goal,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledEuclideanGraphMotionPlannerBase>(robot->species()->motionPlanner());
        return motion_planner->isMemoized(m_index,
                                          robot->species(),
                                          std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(initial),
                                          std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(goal));
    }

    float SubschedulerMotionPlannerInterface::computeTransitionDuration(
        const std::shared_ptr<const ConfigurationBase>& initial,
        const std::shared_ptr<const ConfigurationBase>& goal,
        const std::shared_ptr<const Robot>& robot) const
    {
        auto motion_planner =
            std::dynamic_pointer_cast<SampledEuclideanGraphMotionPlannerBase>(robot->species()->motionPlanner());
        return motion_planner->durationQuery(m_index,
                                             robot->species(),
                                             std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(initial),
                                             std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(goal));
    }
}  // namespace grstapse