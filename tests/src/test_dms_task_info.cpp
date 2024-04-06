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

// External
#include <gtest/gtest.h>
// Project
#include <grstapse/robot.hpp>
#include <grstapse/scheduling/common_scheduler_motion_planner_interface.hpp>
#include <grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler_name_scheme.hpp>
#include <grstapse/scheduling/milp/deterministic/dms_task_info.hpp>
#include <grstapse/task.hpp>
// Local
#include "mock_dms_task_info.hpp"
#include "scheduling_setup.hpp"

namespace grstapse::unittests
{
    TEST(DmsTaskInfo, setupData)
    {
        auto name_scheme                        = std::make_shared<DeterministicMilpSchedulerNameScheme>();
        auto scheduler_motion_planner_interface = std::make_shared<CommonSchedulerMotionPlannerInterface>();

        auto run_test =
            [&name_scheme, &scheduler_motion_planner_interface](const std::string& identifier,
                                                                const PlanOption plan_option,
                                                                const AllocationOption allocation_option,
                                                                const bool homogeneous,
                                                                const std::vector<std::pair<float, float>>& correct)
        {
            auto scheduler_problem_inputs = createSchedulerProblemInputs(plan_option, allocation_option, homogeneous);
            const Eigen::MatrixXf& allocation = scheduler_problem_inputs->allocation();
            const unsigned int num_tasks      = scheduler_problem_inputs->numberOfPlanTasks();

            // Note: for a real problem the task/robot ids will not always line up with their index
            for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
            {
                auto coalition = scheduler_problem_inputs->coalition(task_nr);
                DmsTaskInfo task_info(coalition,
                                      task_nr,
                                      scheduler_problem_inputs->planTask(task_nr),
                                      name_scheme,
                                      scheduler_motion_planner_interface);
                std::shared_ptr<const FailureReason> failure_reason = task_info.setupData();
                ASSERT_FALSE(failure_reason);
                ASSERT_NEAR(task_info.lowerBound(), correct[task_nr].first, 1e-3);
                ASSERT_NEAR(task_info.duration(), correct[task_nr].second, 1e-3);
            }
        };

        run_test("TO",
                 PlanOption::e_total_order,
                 AllocationOption::e_identity,
                 true,
                 {{5.0f, 1.0f}, {5.0f, 7.0f}, {5.0f, 16.0f}});
        run_test("ComplexHomo",
                 PlanOption::e_complex,
                 AllocationOption::e_complex,
                 true,
                 {{5.0f, 1.0f},
                  {7.0710678118654755f, 7.0f},
                  {7.0710678118654755f, 16.0f},
                  {21.213203435596423f, 2.0f},
                  {17.67766952966369f, 8.65685f},
                  {20.113676938839404f, 5.72019f},
                  {47.169905660283014f, 17.2705}});
        run_test("Complex",
                 PlanOption::e_complex,
                 AllocationOption::e_complex,
                 false,
                 {{4.166666666666667f, 1.0f},
                  {5.892556509887896f, 6.16667f},
                  {7.0710678118654755f, 16.f},
                  {17.67766952966369f, 2.f},
                  {14.73139127471974f, 7.71405f},
                  {20.113676938839404f, 5.01682f},
                  {39.30825471690252f, 14.4754f}});
        MilpSolverBase::clearEnvironments();
    }

    TEST(DmsTaskInfo, createTimepointVariables)
    {
        GRBEnv env(true);
        env.start();
        auto name_scheme                        = std::make_shared<DeterministicMilpSchedulerNameScheme>();
        auto scheduler_motion_planner_interface = std::make_shared<CommonSchedulerMotionPlannerInterface>();

        auto run_test = [&env, &name_scheme, &scheduler_motion_planner_interface](const PlanOption plan_option)
        {
            auto scheduler_problem_inputs =
                createSchedulerProblemInputs(plan_option, AllocationOption::e_identity, true);
            const Eigen::MatrixXf& allocation        = scheduler_problem_inputs->allocation();
            const unsigned int num_tasks             = scheduler_problem_inputs->numberOfPlanTasks();
            const unsigned int correct_num_variables = num_tasks;

            GRBModel model(env);
            // Note: for a real problem the task/robot ids will not always line up with their index
            for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
            {
                auto coalition = scheduler_problem_inputs->coalition(task_nr);

                DmsTaskInfo task_info(coalition,
                                      task_nr,
                                      scheduler_problem_inputs->planTask(task_nr),
                                      name_scheme,
                                      scheduler_motion_planner_interface);
                std::shared_ptr<const FailureReason> failure_reason = task_info.setupData();
                ASSERT_FALSE(failure_reason);
                task_info.createTimePointVariables(model);
            }
            model.update();
            ASSERT_EQ(model.get(GRB_IntAttr_NumVars), correct_num_variables);
        };
        run_test(PlanOption::e_total_order);
        run_test(PlanOption::e_branch);
        run_test(PlanOption::e_diamond);
        run_test(PlanOption::e_parallel);
        run_test(PlanOption::e_complex);
        MilpSolverBase::clearEnvironments();
    }

    TEST(DmsTaskInfo, updateLowerBound)
    {
        GRBEnv env(true);
        env.start();
        auto name_scheme                        = std::make_shared<DeterministicMilpSchedulerNameScheme>();
        auto scheduler_motion_planner_interface = std::make_shared<CommonSchedulerMotionPlannerInterface>();

        auto scheduler_problem_inputs =
            createSchedulerProblemInputs(PlanOption::e_total_order, AllocationOption::e_identity, true);
        const Eigen::MatrixXf& allocation        = scheduler_problem_inputs->allocation();
        const unsigned int num_tasks             = scheduler_problem_inputs->numberOfPlanTasks();
        const unsigned int correct_num_variables = num_tasks;

        {
            GRBModel model(env);
            // Note: for a real problem the task/robot ids will not always line up with their index
            const unsigned int task_nr = 0;
            auto coalition             = scheduler_problem_inputs->coalition(task_nr);

            DmsTaskInfo task_info(coalition,
                                  task_nr,
                                  scheduler_problem_inputs->planTask(task_nr),
                                  name_scheme,
                                  scheduler_motion_planner_interface);
            std::shared_ptr<const FailureReason> failure_reason = task_info.setupData();
            ASSERT_FALSE(failure_reason);
            task_info.createTimePointVariables(model);
            task_info.createLowerBoundConstraint(model);
            model.update();

            UpdateModelResult update_model_result = task_info.updateLowerBound(*coalition.begin());
            ASSERT_EQ(update_model_result.type(), UpdateModelResultType::e_no_update);
        }

        {
            GRBModel model(env);
            // Note: for a real problem the task/robot ids will not always line up with their index
            const unsigned int task_nr = 1;
            auto coalition             = scheduler_problem_inputs->coalition(task_nr);

            mocks::MockDmsTaskInfo task_info(coalition,
                                             task_nr,
                                             scheduler_problem_inputs->planTask(task_nr),
                                             name_scheme,
                                             scheduler_motion_planner_interface);
            std::shared_ptr<const FailureReason> failure_reason = task_info.setupData();
            ASSERT_FALSE(failure_reason);
            task_info.setLowerbound(3.0f);  // Actual lowerbound is 5.0f
            task_info.createTimePointVariables(model);
            task_info.createLowerBoundConstraint(model);
            model.update();

            UpdateModelResult update_model_result = task_info.updateLowerBound(*coalition.begin());
            ASSERT_EQ(update_model_result.type(), UpdateModelResultType::e_updated);
        }
    }
}  // namespace grstapse::unittests