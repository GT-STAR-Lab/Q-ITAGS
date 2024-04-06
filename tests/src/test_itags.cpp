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
#include <nlohmann/json.hpp>
// Project
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/config.hpp>
#include <grstapse/problem_inputs/itags_problem_inputs.hpp>
#include <grstapse/task_allocation/itags/itags.hpp>
#include <grstapse/task_allocation/itags/itags_previous_failure_pruning_method.hpp>
#include <grstapse/task_allocation/itags/normalized_allocation_quality.hpp>
#include <grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp>

namespace grstapse::unittests
{
#ifndef NO_MILP
    /*!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, Simple)
    {
        {
            std::ifstream fin(std::string(s_data_dir) + std::string("/problem_inputs/itags/full_run.json"));
            nlohmann::json j;
            fin >> j;
            std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

            Itags itags(problem_inputs, !problem_inputs->useReverse());

            SearchResults<IncrementalTaskAllocationNode, ItagsStatistics> results = itags.search();
            ASSERT_TRUE(results.foundGoal());
            std::shared_ptr<ItagsStatistics> statistics = results.statistics();
            ASSERT_TRUE(statistics);
            fmt::print("time: {}s\n", TimeKeeper::instance().time("itags"));

            std::cout << "Allocation:\n" << results.goal()->allocation() << std::endl;
            std::cout << "Schedule:\n" << results.goal()->schedule()->makespan() << std::endl;
            NormalizedAllocationQuality naq = NormalizedAllocationQuality(problem_inputs);
            std::cout << "Quality:\n" << naq(results.goal()) << std::endl;
        }
        MilpSolverBase::clearEnvironments();
    }

    /*!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, SimpleTETAM)
    {
        {
            std::ifstream fin(std::string(s_data_dir) + std::string("/problem_inputs/itags/full_run_reverse.json"));
            nlohmann::json j;
            fin >> j;
            std::shared_ptr<const ItagsProblemInputs> problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

            Itags itags(problem_inputs, !problem_inputs->useReverse());

            SearchResults<IncrementalTaskAllocationNode, ItagsStatistics> results = itags.search();
            ASSERT_TRUE(results.foundGoal());
            std::shared_ptr<ItagsStatistics> statistics = results.statistics();
            ASSERT_TRUE(statistics);
            fmt::print("time: {}s\n", TimeKeeper::instance().time("itags"));

            std::cout << "Allocation:\n" << results.goal()->allocation() << std::endl;
            std::cout << "Schedule:\n" << results.goal()->schedule()->makespan() << std::endl;
            NormalizedAllocationQuality naq = NormalizedAllocationQuality(problem_inputs);
            std::cout << "Quality:\n" << naq(results.goal()) << std::endl;
        }
        MilpSolverBase::clearEnvironments();
    }

    /*!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, Hard) {}

    /*!
     * TODO(Andrew,Glen): better tests
     */
    TEST(Itags, WriteSolution)
    {
        {
            std::ifstream fin(std::string(s_data_dir) + std::string("/problem_inputs/itags/full_run.json"));
            nlohmann::json j;
            fin >> j;
            auto problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
            Itags itags(problem_inputs);

            SearchResults<IncrementalTaskAllocationNode, ItagsStatistics> results = itags.search();
            results.writeToFile("itags_test_output.json", problem_inputs);
        }
        MilpSolverBase::clearEnvironments();
    }
#endif
}  // namespace grstapse::unittests