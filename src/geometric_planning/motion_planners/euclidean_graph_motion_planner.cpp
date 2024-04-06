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
#include "grstapse/geometric_planning/motion_planners/euclidean_graph_motion_planner.hpp"

// Local
#include "grstapse/common/search/undirected_graph/undirected_graph_path_cost.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_successor_generator.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/miscellaneous/equal_euclidean_graph_configuration_goal_check.hpp"
#include "grstapse/geometric_planning/miscellaneous/euclidean_graph_a_star.hpp"
#include "grstapse/geometric_planning/miscellaneous/euclidean_graph_configuration_euclidean_distance_heuristic.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"
#include "grstapse/geometric_planning/query_results/euclidean_graph_motion_planner_query_result.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/parameters/parameters_factory.hpp"

namespace grstapse
{
    EuclideanGraphMotionPlanner::EuclideanGraphMotionPlanner(const std::shared_ptr<const ParametersBase>& parameters,
                                                             const std::shared_ptr<EuclideanGraphEnvironment>& graph)
        : SingularEuclideanGraphMotionPlannerBase(parameters, graph)
        , m_search_parameters(ParametersFactory::instance().create(
              ParametersFactory::Type::e_search,
              {{constants::k_config_type, constants::k_best_first_search_parameters},
               {constants::k_has_timeout, parameters->get<float>(constants::k_timeout) > 0.0f},
               {constants::k_timeout, parameters->get<float>(constants::k_timeout)},
               {constants::k_timer_name, constants::k_motion_planning_time + std::string("_a_star")}}))
        , m_astar_functors({
              .path_cost           = std::make_shared<const UndirectedGraphPathCost<SearchNode>>(),
              .heuristic           = nullptr,  // Gets set for each A* search individually
              .successor_generator = std::make_shared<const UndirectedGraphSuccessorGenerator<SearchNode>>(graph),
              .goal_check          = nullptr  // Gets set for each A* search individually
          })
    {}

    std::shared_ptr<const MotionPlannerQueryResultBase> EuclideanGraphMotionPlanner::computeMotionPlan(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        auto ic = std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(initial_configuration);
        auto gc = std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(goal_configuration);

        m_astar_functors.heuristic =
            std::make_shared<const EuclideanGraphConfigurationEuclideanDistanceHeuristic<SearchNode>>(gc);
        m_astar_functors.goal_check = std::make_shared<const EqualEuclideanGraphConfigurationGoalCheck<SearchNode>>(gc);

        EuclideanGraphAStar a_star(m_search_parameters,
                                   ic,
                                   std::dynamic_pointer_cast<EuclideanGraphEnvironment>(m_environment),
                                   m_astar_functors);
        auto result = a_star.search();
        if(!result.foundGoal())
        {
            return std::make_shared<EuclideanGraphMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_timeout);
        }

        std::vector<std::shared_ptr<const EuclideanGraphConfiguration>> path;
        traceApply<SearchNode>(result.goal(),
                               [&path](const std::shared_ptr<const SearchNode>& node)
                               {
                                   path.push_back(node->vertex()->payload());
                               });
        std::reverse(path.begin(), path.end());

        return std::make_shared<EuclideanGraphMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_success, path);
    }

}  // namespace grstapse