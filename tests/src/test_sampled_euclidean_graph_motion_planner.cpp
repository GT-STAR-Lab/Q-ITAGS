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
#include <nlohmann/json.hpp>
// Project
#include <grstapse/common/utilities/json_tree_factory.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp>
#include <grstapse/geometric_planning/environments/euclidean_graph_environment.hpp>
#include <grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp>
#include <grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner.hpp>
#include <grstapse/geometric_planning/motion_planning_enums.hpp>
#include <grstapse/geometric_planning/query_results/euclidean_graph_motion_planner_query_result.hpp>
#include <grstapse/parameters/parameters_base.hpp>
#include <grstapse/parameters/parameters_factory.hpp>

namespace grstapse::unittests
{
    TEST(SampledEuclideanGraphMotionPlanner, Simple)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_euclidean_graph_motion_planner_parameters},
                           {constants::k_is_complete, false},
                           {constants::k_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) +
                          std::string("/geometric_planning/environments/sampled_euclidean_graph.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<SampledEuclideanGraphEnvironment>>();

        auto motion_planner = std::make_shared<SampledEuclideanGraphMotionPlanner>(parameters, environment);

        auto ic = std::make_shared<EuclideanGraphConfiguration>(0, 0.0f, 0.0f);
        auto gc = std::make_shared<EuclideanGraphConfiguration>(18, 4.0f, 4.0f);

        {
            auto result = motion_planner->query(0, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const EuclideanGraphMotionPlannerQueryResult>(result)->path();
            ASSERT_EQ(path.size(), 9);
        }
        {
            auto result = motion_planner->query(1, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const EuclideanGraphMotionPlannerQueryResult>(result)->path();
            ASSERT_EQ(path.size(), 9);
        }
        {
            auto result = motion_planner->query(2, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const EuclideanGraphMotionPlannerQueryResult>(result)->path();
            ASSERT_EQ(path.size(), 11);
        }
    }

    TEST(SampledEuclideanGraphMotionPlanner, Load)
    {
        std::ifstream fin(std::string(s_data_dir) +
                          std::string("/problem_inputs/grstaps/sampled_euclidean_graph_motion_planners.json"));
        nlohmann::json j;
        fin >> j;

        MotionPlannerBase::init();
        auto motion_planner = std::dynamic_pointer_cast<SampledEuclideanGraphMotionPlanner>(
            JsonTreeFactory<MotionPlannerBase>::instance().create(j[0]));

        auto ic = std::make_shared<EuclideanGraphConfiguration>(55, 754.0f, 426.0f);
        auto gc = std::make_shared<EuclideanGraphConfiguration>(39, 712.0f, 144.0f);
        for(unsigned int i = 0; i < 10; ++i)
        {
            auto result = motion_planner->query(i, nullptr, ic, gc);
            auto path   = std::dynamic_pointer_cast<const EuclideanGraphMotionPlannerQueryResult>(result)->path();
        }
    }
}  // namespace grstapse::unittests