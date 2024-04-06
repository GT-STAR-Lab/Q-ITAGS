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
#include <memory>
// External
#include <gtest/gtest.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// Project
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/configurations/se2_state_ompl_configuration.hpp>
#include <grstapse/geometric_planning/environments/pgm_ompl_environment.hpp>
#include <grstapse/geometric_planning/motion_planners/ompl_motion_planner.hpp>
#include <grstapse/geometric_planning/motion_planning_enums.hpp>
#include <grstapse/geometric_planning/query_results/ompl_motion_planner_query_result.hpp>
#include <grstapse/parameters/parameters_base.hpp>
#include <grstapse/parameters/parameters_factory.hpp>

namespace grstapse::unittests
{
    TEST(MotionPlanner, MemoizationState)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;
        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_EQ(path, std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, MemoizationStateDifferentRadius)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.3f, 0.2f, motion_planner);
            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_NE(path, std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, MemoizationStateDifferentStartState)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;
        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(4.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_NE(path, std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path());
        }
    }

    TEST(MotionPlanner, MemoizationStateDifferentState)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        std::shared_ptr<const ompl::geometric::PathGeometric> path;
        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);

        // Run 1
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        }

        // Run 2
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
            auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-4.5, 0.0, 3.14159);

            std::shared_ptr<const MotionPlannerQueryResultBase> result =
                motion_planner->query(species, initial_configuration, goal_configuration);
            ASSERT_NE(path, std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path());
        }
    }

    void basicMpTest(OmplMotionPlannerType t)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, t},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner = std::make_shared<OmplMotionPlanner>(t, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(5.5, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-5.5, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlannerQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        ASSERT_EQ(path->getStateCount(), 2);
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 5.5);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(1));
        ASSERT_FLOAT_EQ(state_1->getX(), -5.5);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, PrmStar)
    {
        basicMpTest(OmplMotionPlannerType::e_prm_star);
    }

    TEST(MotionPlanner, LazyPrm)
    {
        basicMpTest(OmplMotionPlannerType::e_lazy_prm);
    }

    TEST(MotionPlanner, LazyPrmStar)
    {
        basicMpTest(OmplMotionPlannerType::e_lazy_prm_star);
    }

    TEST(MotionPlanner, Rrt)
    {
        basicMpTest(OmplMotionPlannerType::e_rrt);
    }

    TEST(MotionPlanner, RrtStar)
    {
        basicMpTest(OmplMotionPlannerType::e_rrt_star);
    }

    TEST(MotionPlanner, RrtConnect)
    {
        basicMpTest(OmplMotionPlannerType::e_rrt_connect);
    }

    TEST(MotionPlanner, ParallelRrt)
    {
        basicMpTest(OmplMotionPlannerType::e_parallel_rrt);
    }

    TEST(MotionPlanner, LazyRrt)
    {
        basicMpTest(OmplMotionPlannerType::e_lazy_rrt);
    }

    TEST(MotionPlanner, CenterBlock)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) +
                          std::string("/geometric_planning/environments/pgm_center_block.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(50.0, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-50.0, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlannerQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        environment->setSpecies(species);
        ASSERT_TRUE(path->getStateCount() > 2);
        for(unsigned int i = 0; i < path->getStateCount(); ++i)
        {
            ASSERT_TRUE(environment->isValid(path->getState(i)));
        }
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 50.0);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 =
            dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(path->getStateCount() - 1));
        ASSERT_FLOAT_EQ(state_1->getX(), -50.0);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, Wall)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_wall.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(50.0, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-50.0, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlannerQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);

        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_success);

        const auto& path = std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->path();
        environment->setSpecies(species);
        ASSERT_TRUE(path->getStateCount() > 2);
        for(unsigned int i = 0; i < path->getStateCount(); ++i)
        {
            ASSERT_TRUE(environment->isValid(path->getState(i)));
        }
        const auto* state_0 = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(0));
        ASSERT_FLOAT_EQ(state_0->getX(), 50.0);
        ASSERT_FLOAT_EQ(state_0->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_0->getYaw(), 3.14159);
        const auto* state_1 =
            dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(path->getState(path->getStateCount() - 1));
        ASSERT_FLOAT_EQ(state_1->getX(), -50.0);
        ASSERT_FLOAT_EQ(state_1->getY(), 0.0);
        ASSERT_FLOAT_EQ(state_1->getYaw(), 3.14159);
    }

    TEST(MotionPlanner, Segmented)
    {
        auto parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});

        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_segmented.json"));

        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        auto motion_planner =
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, parameters, environment);

        auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(50.0, 0.0, 3.14159);
        auto goal_configuration    = std::make_shared<Se2StateOmplConfiguration>(-50.0, 0.0, 3.14159);

        auto species = std::make_shared<Species>("name", Eigen::VectorXf{}, 0.2f, 0.2f, motion_planner);
        std::shared_ptr<const MotionPlannerQueryResultBase> result =
            motion_planner->query(species, initial_configuration, goal_configuration);
        ASSERT_EQ(result->status(), MotionPlannerQueryStatus::e_failure);
    }
}  // namespace grstapse::unittests