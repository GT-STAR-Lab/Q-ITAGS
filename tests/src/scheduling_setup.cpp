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
#include "scheduling_setup.hpp"

// Global
#include <fstream>
// External
#include <fmt/format.h>
// Project
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/environments/pgm_ompl_environment.hpp>
#include <grstapse/geometric_planning/motion_planning_enums.hpp>
#include <grstapse/parameters/parameters_base.hpp>
#include <grstapse/parameters/parameters_factory.hpp>
// Local
#include "mock_normalized_schedule_quality.hpp"

namespace grstapse::unittests
{
    void createTotalOrderPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                    std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints)
    {
        auto t1_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 1.0f, 0.0f);
        auto t3_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 1.0f, 0.0f);

        auto t1_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 2.0f, 0.0f);
        auto t3_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 4.0f, 0.0f);

        // Note: desired traits are ignored by the scheduler so we don't create them
        tasks                  = {std::make_shared<const Task>(std::make_shared<SasAction>("t1", 1.0f),
                                              Eigen::VectorXf(),
                                              t1_initial_configuration,
                                              t1_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t2", 2.0f),
                                              Eigen::VectorXf(),
                                              t2_initial_configuration,
                                              t2_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t3", 1.0f),
                                              Eigen::VectorXf(),
                                              t3_initial_configuration,
                                              t3_terminal_configuration)};
        precedence_constraints = {{0, 1}, {0, 2}, {1, 2}};
    }
    void createBranchPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints)
    {
        auto t1_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 1.0f, 0.0f);
        auto t3_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 1.0f, 0.0f);

        auto t1_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 2.0f, 0.0f);
        auto t3_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 4.0f, 0.0f);

        // Note: desired traits are ignored by the scheduler, so we don't create them
        tasks                  = {std::make_shared<const Task>(std::make_shared<SasAction>("t1", 1.0f),
                                              Eigen::VectorXf(),
                                              t1_initial_configuration,
                                              t1_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t2", 2.0f),
                                              Eigen::VectorXf(),
                                              t2_initial_configuration,
                                              t2_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t3", 1.0f),
                                              Eigen::VectorXf(),
                                              t3_initial_configuration,
                                              t3_terminal_configuration)};
        precedence_constraints = {{0, 1}, {0, 2}};
    }

    void createDiamondPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                 std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints)
    {
        auto t1_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 1.0f, 0.0f);
        auto t3_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 1.0f, 0.0f);
        auto t4_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 3.0f, 0.0f);

        auto t1_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 2.0f, 0.0f);
        auto t3_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 4.0f, 0.0f);
        auto t4_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 3.0f, 0.0f);

        // Note: desired Traits are ignored by the scheduler, so we don't create them
        tasks                  = {std::make_shared<const Task>(std::make_shared<SasAction>("t1", 1.0f),
                                              Eigen::VectorXf(),
                                              t1_initial_configuration,
                                              t1_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t2", 2.0f),
                                              Eigen::VectorXf(),
                                              t2_initial_configuration,
                                              t2_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t3", 1.0f),
                                              Eigen::VectorXf(),
                                              t3_initial_configuration,
                                              t3_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t4", 2.0f),
                                              Eigen::VectorXf(),
                                              t4_initial_configuration,
                                              t4_terminal_configuration)};
        precedence_constraints = {{0, 1}, {0, 2}, {0, 3}, {1, 3}, {2, 3}};
    }

    void createParallelPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                  std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints)
    {
        auto t1_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 1.0f, 0.0f);
        auto t3_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 1.0f, 0.0f);
        auto t4_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 3.0f, 0.0f);

        auto t1_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 2.0f, 0.0f);
        auto t3_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 4.0f, 0.0f);
        auto t4_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 3.0f, 0.0f);

        // Note: desired Traits are ignored by the scheduler, so we don't create them
        tasks                  = {std::make_shared<const Task>(std::make_shared<SasAction>("t1", 1.0f),
                                              Eigen::VectorXf(),
                                              t1_initial_configuration,
                                              t1_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t2", 2.0f),
                                              Eigen::VectorXf(),
                                              t2_initial_configuration,
                                              t2_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t3", 1.0f),
                                              Eigen::VectorXf(),
                                              t3_initial_configuration,
                                              t3_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t4", 2.0f),
                                              Eigen::VectorXf(),
                                              t4_initial_configuration,
                                              t4_terminal_configuration)};
        precedence_constraints = {{0, 1}, {2, 3}};
    }
    void createComplexPlanInputs(std::vector<std::shared_ptr<const Task>>& tasks,
                                 std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints)
    {
        auto t1_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 1.0f, 0.0f);
        auto t3_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 1.0f, 0.0f);
        auto t4_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 3.0f, 0.0f);
        auto t5_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(2.5f, 2.5f, 0.0f);
        auto t6_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(3.68f, 3.0f, 0.0f);
        auto t7_initial_configuration = std::make_shared<Se2StateOmplConfiguration>(10.0f, 5.0f, 0.0f);

        auto t1_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(0.0f, 1.0f, 0.0f);
        auto t2_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(1.0f, 2.0f, 0.0f);
        auto t3_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(2.0f, 4.0f, 0.0f);
        auto t4_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 3.0f, 0.0f);
        auto t5_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(1.7f, 1.7f, 0.0f);
        auto t6_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(3.0f, 2.5f, 0.0f);
        auto t7_terminal_configuration = std::make_shared<Se2StateOmplConfiguration>(7.0f, 3.5f, 0.0f);

        // Note: desired Traits are ignored by the scheduler, so we don't create them
        tasks = {std::make_shared<const Task>(std::make_shared<SasAction>("t1", 1.0f),
                                              Eigen::VectorXf(),
                                              t1_initial_configuration,
                                              t1_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t2", 2.0f),
                                              Eigen::VectorXf(),
                                              t2_initial_configuration,
                                              t2_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t3", 1.0f),
                                              Eigen::VectorXf(),
                                              t3_initial_configuration,
                                              t3_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t4", 2.0f),
                                              Eigen::VectorXf(),
                                              t4_initial_configuration,
                                              t4_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t5", 3.0f),
                                              Eigen::VectorXf(),
                                              t5_initial_configuration,
                                              t5_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t6", 1.5f),
                                              Eigen::VectorXf(),
                                              t6_initial_configuration,
                                              t6_terminal_configuration),
                 std::make_shared<const Task>(std::make_shared<SasAction>("t4", 0.5f),
                                              Eigen::VectorXf(),
                                              t7_initial_configuration,
                                              t7_terminal_configuration)};
        precedence_constraints =
            {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 4}, {5, 2}, {5, 3}, {5, 4}, {5, 6}};
    }

    void createHomogeneousRobots(SpeciesOption species_option,
                                 unsigned int num,
                                 std::vector<std::shared_ptr<const Robot>>& robots,
                                 std::vector<std::shared_ptr<const Species>>& species,
                                 std::vector<std::shared_ptr<MotionPlannerBase>>& motion_planners)
    {
        // Environment
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        // Motion Planner
        auto motion_planner_parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});
        motion_planners.push_back(
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, motion_planner_parameters, environment));

        switch(species_option)
        {
            case SpeciesOption::e_burger:
            {
                species.push_back(
                    std::make_shared<const Species>("burger", Eigen::VectorXf(), 0.2f, 0.2f, motion_planners.front()));
                break;
            }
            case SpeciesOption::e_waffle:
            {
                species.push_back(std::make_shared<const Species>("waffle",
                                                                  Eigen::VectorXf(),
                                                                  0.32f,
                                                                  0.24f,
                                                                  motion_planners.front()));
                break;
            }
        }

        robots.reserve(num);
        for(unsigned int i = 0; i < num; ++i)
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(i * 1.0f, 0.0f, 0.f);
            robots.push_back(
                std::make_shared<const Robot>(fmt::format("r{0:d}", i), initial_configuration, species.front()));
        }
    }
    void createHeterogeneousRobots(const std::vector<SpeciesOption>& species_options,
                                   std::vector<std::shared_ptr<const Robot>>& robots,
                                   std::vector<std::shared_ptr<const Species>>& species,
                                   std::vector<std::shared_ptr<MotionPlannerBase>>& motion_planners)
    {
        // Environment
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        // Motion Planner
        auto motion_planner_parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_motion_planner,
            nlohmann::json{{constants::k_config_type, constants::k_ompl_motion_planner_parameters},
                           {constants::k_ompl_mp_algorithm, OmplMotionPlannerType::e_prm},
                           {constants::k_timeout, 0.1f},
                           {constants::k_simplify_path, true},
                           {constants::k_simplify_path_timeout, 0.1f}});
        motion_planners.push_back(
            std::make_shared<OmplMotionPlanner>(OmplMotionPlannerType::e_prm, motion_planner_parameters, environment));

        // Species
        species.push_back(
            std::make_shared<const Species>("burger", Eigen::VectorXf(), 0.2f, 0.2f, motion_planners.front()));
        species.push_back(
            std::make_shared<const Species>("waffle", Eigen::VectorXf(), 0.32f, 0.24f, motion_planners.front()));

        robots.reserve(species_options.size());
        for(unsigned int i = 0; i < species_options.size(); ++i)
        {
            auto initial_configuration = std::make_shared<Se2StateOmplConfiguration>(i * 1.0f, 0.0f, 0.f);
            switch(species_options[i])
            {
                case SpeciesOption::e_burger:
                {
                    robots.push_back(
                        std::make_shared<const Robot>(fmt::format("r{0:d}", i), initial_configuration, species[0]));
                    break;
                }
                case SpeciesOption::e_waffle:
                {
                    robots.push_back(
                        std::make_shared<const Robot>(fmt::format("r{0:d}", i), initial_configuration, species[1]));
                    break;
                }
            }
        }
    }
    std::shared_ptr<SchedulerProblemInputs> createSchedulerProblemInputs(PlanOption plan_option,
                                                                         AllocationOption allocation_option,
                                                                         bool homogeneous)
    {
        // region Grstaps Problem Inputs
        std::vector<std::shared_ptr<const Task>> tasks;
        std::set<std::pair<unsigned int, unsigned int>> precedence_constraints;
        switch(plan_option)
        {
            case PlanOption::e_total_order:
            {
                createTotalOrderPlanInputs(tasks, precedence_constraints);
                break;
            }
            case PlanOption::e_branch:
            {
                createBranchPlanInputs(tasks, precedence_constraints);
                break;
            }
            case PlanOption::e_diamond:
            {
                createDiamondPlanInputs(tasks, precedence_constraints);
                break;
            }
            case PlanOption::e_parallel:
            {
                createParallelPlanInputs(tasks, precedence_constraints);
                break;
            }
            case PlanOption::e_complex:
            {
                createComplexPlanInputs(tasks, precedence_constraints);
                break;
            }
        }
        const unsigned int num_tasks = tasks.size();

        unsigned int num_robots;
        switch(allocation_option)
        {
            case AllocationOption::e_none:
            {
                num_robots = num_tasks;
                break;
            }
            case AllocationOption::e_identity:
            {
                num_robots = num_tasks;
                break;
            }
            case AllocationOption::e_multi_task_robot:
            {
                // A robot gets assigned two tasks (because there is one more task than robots)
                num_robots = num_tasks - 1;
                break;
            }
            case AllocationOption::e_multi_robot_task:
            {
                // A task gets assigned two robots (because there is one more robot than tasks)
                num_robots = num_tasks + 1;
                break;
            }
            case AllocationOption::e_complex:
                [[fallthrough]];
            case AllocationOption::e_complex2:
            {
                assert(plan_option == PlanOption::e_complex);
                num_robots = 3;
                break;
            }
        }
        std::vector<std::shared_ptr<const Species>> species;
        std::vector<std::shared_ptr<const Robot>> robots;
        std::vector<std::shared_ptr<MotionPlannerBase>> motion_planners;
        if(homogeneous)
        {
            createHomogeneousRobots(SpeciesOption::e_burger, num_robots, robots, species, motion_planners);
        }
        else
        {
            // Alternate between burger and waffle (turtlebots)
            std::vector<SpeciesOption> species_options;
            for(unsigned int i = 0; i < num_robots; ++i)
            {
                species_options.push_back(i % 2 ? SpeciesOption::e_burger : SpeciesOption::e_waffle);
            }
            createHeterogeneousRobots(species_options, robots, species, motion_planners);
        }

        auto grstaps_problem_inputs = std::make_shared<mocks::MockGrstapsProblemInputs>();
        grstaps_problem_inputs->setSpecies(species);
        grstaps_problem_inputs->setTasks(tasks);
        grstaps_problem_inputs->setRobots(robots);
        grstaps_problem_inputs->setMotionPlanners(motion_planners);

        auto schedule_parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_scheduler,
            nlohmann::json{{constants::k_config_type, constants::k_deterministic_milp_scheduler_parameters},
                           {constants::k_timeout, 1.0f},
                           {constants::k_milp_timeout, 1.0f},
                           {constants::k_threads, 0u},
                           {constants::k_use_hierarchical_objective, true}});
        grstaps_problem_inputs->setScheduleParameters(schedule_parameters);
        // endregion

        // region Itags Problem Inputs
        //  Note: desired_traits and makespan_best are not used by MilpScheduler, so they are ignored
        auto itags_problem_inputs =
            std::make_shared<ItagsProblemInputs>(grstaps_problem_inputs, precedence_constraints);
        // endregion

        Eigen::MatrixXf allocation(num_tasks, num_robots);
        allocation.setZero();
        switch(allocation_option)
        {
            case AllocationOption::e_none:
            {
                break;
            }
            case AllocationOption::e_identity:
            {
                for(unsigned int i = 0; i < num_tasks; ++i)
                {
                    allocation(i, i) = 1.0f;
                }
                break;
            }
            case AllocationOption::e_multi_task_robot:
            {
                // There is a robot that gets assigned two tasks
                for(unsigned int i = 0; i < num_tasks; ++i)
                {
                    allocation(i, i % num_robots) = 1.0f;
                }
                break;
            }
            case AllocationOption::e_multi_robot_task:
            {
                // There is a task that gets assigned two robots
                for(unsigned int i = 0; i < num_robots; ++i)
                {
                    allocation(i % num_tasks, i) = 1.0f;
                }
                break;
            }
            case AllocationOption::e_complex:
            {
                assert(plan_option == PlanOption::e_complex);
                allocation(0, 0) = 1.0f;
                allocation(1, 0) = 1.0f;
                allocation(3, 0) = 1.0f;
                allocation(4, 0) = 1.0f;

                allocation(5, 1) = 1.0f;
                allocation(2, 1) = 1.0f;

                allocation(5, 2) = 1.0f;
                allocation(6, 2) = 1.0f;
                break;
            }
            case AllocationOption::e_complex2:
            {
                assert(plan_option == PlanOption::e_complex);
                allocation(0, 0) = 1.0f;
                allocation(6, 0) = 1.0f;
                allocation(2, 0) = 1.0f;

                allocation(1, 1) = 1.0f;
                allocation(5, 1) = 1.0f;
                allocation(3, 1) = 1.0f;

                allocation(2, 2) = 1.0f;
                allocation(4, 2) = 1.0f;
                break;
            }
        }

        return std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);
    }
}  // namespace grstapse::unittests