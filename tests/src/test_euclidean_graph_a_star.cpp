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

// Global
#include <fstream>
// External
#include <gtest/gtest.h>
// Project
#include <grstapse/common/search/undirected_graph/undirected_graph_a_star_search_node.hpp>
#include <grstapse/common/search/undirected_graph/undirected_graph_path_cost.hpp>
#include <grstapse/common/search/undirected_graph/undirected_graph_successor_generator.hpp>
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/environments/euclidean_graph_environment.hpp>
#include <grstapse/geometric_planning/miscellaneous/equal_euclidean_graph_configuration_goal_check.hpp>
#include <grstapse/geometric_planning/miscellaneous/euclidean_graph_a_star.hpp>
#include <grstapse/geometric_planning/miscellaneous/euclidean_graph_configuration_euclidean_distance_heuristic.hpp>
#include <grstapse/parameters/parameters_factory.hpp>

namespace grstapse::unittests
{
    TEST(EuclideanGraphAStar, Simple)
    {
        using SearchNode = UndirectedGraphAStarSearchNode<EuclideanGraphConfiguration>;

        auto search_parameters =
            ParametersFactory::instance().create(ParametersFactory::Type::e_search,
                                                 {{constants::k_config_type, constants::k_best_first_search_parameters},
                                                  {constants::k_has_timeout, false},
                                                  {constants::k_timeout, 0.0f},
                                                  {constants::k_timer_name, "euclidean_graph_a_star"}});
        std::ifstream in(std::string(s_data_dir) +
                         std::string("/geometric_planning/environments/euclidean_graph.json"));
        nlohmann::json j;
        in >> j;

        auto graph                 = j.get<std::shared_ptr<EuclideanGraphEnvironment>>();
        auto initial_configuration = std::make_shared<const EuclideanGraphConfiguration>(0, 0.0f, 0.0f);
        auto goal_configuration    = std::make_shared<const EuclideanGraphConfiguration>(18, 4.0f, 4.0f);

        AStarFunctors<SearchNode> functors(
            {.path_cost = std::make_shared<const UndirectedGraphPathCost<SearchNode>>(),
             .heuristic = std::make_shared<const EuclideanGraphConfigurationEuclideanDistanceHeuristic<SearchNode>>(
                 goal_configuration),
             .successor_generator = std::make_shared<const UndirectedGraphSuccessorGenerator<SearchNode>>(graph),
             .goal_check =
                 std::make_shared<const EqualEuclideanGraphConfigurationGoalCheck<SearchNode>>(goal_configuration)});

        EuclideanGraphAStar a_star(search_parameters, initial_configuration, graph, functors);
        auto results = a_star.search();
        auto path    = trace<SearchNode>(results.goal());
        ASSERT_TRUE(results.foundGoal());
        ASSERT_EQ(path.size(), 9);
    }

}  // namespace grstapse::unittests