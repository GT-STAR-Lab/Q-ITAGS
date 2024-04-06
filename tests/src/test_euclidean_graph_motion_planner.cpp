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
// Local
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/common/utilities/json_tree_factory.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/environments/euclidean_graph_environment.hpp>
#include <grstapse/geometric_planning/motion_planners/euclidean_graph_motion_planner.hpp>
#include <grstapse/geometric_planning/motion_planning_enums.hpp>
#include <grstapse/geometric_planning/query_results/euclidean_graph_motion_planner_query_result.hpp>
#include <grstapse/parameters/parameters_base.hpp>
#include <grstapse/parameters/parameters_factory.hpp>

namespace grstapse::unittests
{
    TEST(EuclideanGraphMotionPlanner, Simple)
    {
        std::ifstream in(std::string(s_data_dir) +
                         std::string("/geometric_planning/environments/euclidean_graph.json"));
        nlohmann::json j;
        in >> j;

        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{
                {constants::k_config_type, constants::k_euclidean_graph_motion_planner_parameters},
                {constants::k_is_complete, true},
                {constants::k_timeout, 1.0f}  // s
            });
        auto graph = j.get<std::shared_ptr<EuclideanGraphEnvironment>>();
        EuclideanGraphMotionPlanner mp(parameters, graph);

        auto initial_configuration = std::make_shared<const EuclideanGraphConfiguration>(0, 0.0f, 0.0f);
        auto goal_configuration    = std::make_shared<const EuclideanGraphConfiguration>(18, 4.0f, 4.0f);
        auto result                = std::dynamic_pointer_cast<const EuclideanGraphMotionPlannerQueryResult>(
            mp.query(nullptr, initial_configuration, goal_configuration));
        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);
        auto path = result->path();
        ASSERT_EQ(path.size(), 9);
    }

    TEST(EuclideanGraphMotionPlanner, Load)
    {
        std::ifstream in(std::string(s_data_dir) +
                         std::string("/problem_inputs/grstaps/euclidean_graph_motion_planners.json"));
        nlohmann::json j;
        in >> j;

        MotionPlannerBase::init();
        auto mp = std::dynamic_pointer_cast<EuclideanGraphMotionPlanner>(
            JsonTreeFactory<MotionPlannerBase>::instance().create(j[0]));

        auto initial_configuration = std::make_shared<const EuclideanGraphConfiguration>(0, 0.0f, 0.0f);
        auto goal_configuration    = std::make_shared<const EuclideanGraphConfiguration>(18, 4.0f, 4.0f);
        auto result                = std::dynamic_pointer_cast<const EuclideanGraphMotionPlannerQueryResult>(
            mp->query(nullptr, initial_configuration, goal_configuration));
        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);
        auto path = result->path();
        ASSERT_EQ(path.size(), 9);
    }
}  // namespace grstapse::unittests