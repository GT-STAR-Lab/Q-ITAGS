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
#include <nlohmann/json.hpp>
// Project
#include <grstapse/core.hpp>

namespace grstapse::main
{
    void glen_ditags_scheduling()
    {
        std::ifstream in("data/itags_problems/ditags_survivor_problem0.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 20, 6> allocation;
        allocation << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule  = scheduler.solve();
        float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time   = smp_time - mp_time;
        fmt::print("MP Time: {0:f}s\nS Time: {1:f}s\nNum MILPs run: {2:d}\n",
                   mp_time,
                   s_time,
                   MilpSchedulerBase::numIterations());
    }

    void glen_ditags_ta()
    {
        std::ifstream in("data/itags_problems/ditags_survivor_problem0.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
        Itags itags(itags_problem_inputs);
        auto result      = itags.search();
        float mp_time    = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time   = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time     = smp_time - mp_time;
        float total_time = TimeKeeper::instance().time("itags");
        float ta_time    = total_time - smp_time;
        fmt::print("MP Time: {0:f}s\nS Time: {1:f}s\nTA Time: {2:f}\nTotal Time: {3:f}\nNum MILPs run: {4:d}\n",
                   mp_time,
                   s_time,
                   ta_time,
                   total_time,
                   MilpSchedulerBase::numIterations());
    }

    void glen_ditags_ta2()
    {
        std::ifstream in("data/ditags_survivor_problem0.json");
        nlohmann::json j;
        in >> j;

        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        ItagsBuilderOptions builder_options{
            .prepruning = {ItagsBuilderOptions::PrepruningMethodOptions::e_no_trait_improvement,
                           ItagsBuilderOptions::PrepruningMethodOptions::e_previous_failure_reason}};
        ItagsBuilder builder(builder_options);
        std::shared_ptr<Itags> itags = builder.build(itags_problem_inputs);

        auto result      = itags->search();
        float mp_time    = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time   = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time     = smp_time - mp_time;
        float total_time = TimeKeeper::instance().time("itags");
        float ta_time    = total_time - smp_time;
        fmt::print("MP Time: {0:f}s\nS Time: {1:f}s\nTA Time: {2:f}\nTotal Time: {3:f}\nNum MILPs run: {4:d}\n",
                   mp_time,
                   s_time,
                   ta_time,
                   total_time,
                   MilpSchedulerBase::numIterations());
        result.writeToFile("glen_ditags_2.json", itags_problem_inputs);
    }

    void stochastic_scheduling_heuristic_10_10_5()
    {
        std::ifstream in("data/itags_problems/itags_heuristic_polypixel_400maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 10, 5> allocation;
        allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        HeuristicApproximationStochasticScheduler scheduler(scheduler_problem_inputs);
        auto schedule          = scheduler.solve();
        const float mp_time    = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float smp_time   = TimeKeeper::instance().time(constants::k_scheduling_time);
        const float s_time     = smp_time - mp_time;
        const float label_time = TimeKeeper::instance().time("label");
        const float milp_time  = TimeKeeper::instance().time("milp");
        const float sprt_time  = TimeKeeper::instance().time("sprt");
        fmt::print("HA:\n\tTime: {0:f}s\n\tLabel: {2:f}s\n\tMilp: {3:f}s\n\tSprt: {4:f}s\n\t\n\tMakespan: {1:f}\n",
                   s_time,
                   schedule->schedule()->makespan(),
                   label_time,
                   milp_time,
                   sprt_time);
    }

    void stochastic_scheduling_mono_10_10_5()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 10, 5> allocation;
        allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        MonolithicStochasticMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule  = scheduler.solve();
        float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time   = smp_time - mp_time;
        fmt::print("Mono:\n\tTime: {0:f}s\n\tMakespan: {1:f}\n", s_time, schedule->schedule()->makespan());
    }

    void stochastic_scheduling_bender_10_10_5()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 10, 5> allocation;
        allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        BendersStochasticMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule  = scheduler.solve();
        float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time   = smp_time - mp_time;
        fmt::print("Bender:\n\tTime: {0:f}s\n\tMakespan: {1:f}\n", s_time, schedule->schedule()->makespan());
    }

    void stochastic_scheduling_bender_parallel_10_10_5()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 10, 5> allocation;
        allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        BendersParallelStochasticMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule  = scheduler.solve();
        float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        float s_time   = smp_time - mp_time;
        fmt::print("Bender Parallel:\n\tTime: {0:f}s\n\tMakespan: {1:f}\n", s_time, schedule->schedule()->makespan());
    }

    void stochastic_scheduling_10_10_5()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 10, 5> allocation;
        allocation << 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        MilpSolverBase::clearEnvironments();
        {
            MonolithicStochasticMilpScheduler scheduler(scheduler_problem_inputs);
            auto schedule  = scheduler.solve();
            float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
            float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
            float s_time   = smp_time - mp_time;
            fmt::print("Mono:\n\tTime: {0:f}s\n\tMakespan: {1:f}\n", s_time, schedule->schedule()->makespan());
        }
        MilpSolverBase::clearEnvironments();
        {
            BendersStochasticMilpScheduler scheduler(scheduler_problem_inputs);
            auto schedule  = scheduler.solve();
            float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
            float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
            float s_time   = smp_time - mp_time;
            fmt::print("Bender:\n\tTime: {0:f}s\n\tMakespan: {1:f}\n", s_time, schedule->schedule()->makespan());
        }
        MilpSolverBase::clearEnvironments();
        {
            BendersParallelStochasticMilpScheduler scheduler(scheduler_problem_inputs);
            auto schedule  = scheduler.solve();
            float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
            float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
            float s_time   = smp_time - mp_time;
            fmt::print("Bender Parallel:\n\tTime: {0:f}s\n\tMakespan: {1:f}\n",
                       s_time,
                       schedule->schedule()->makespan());
        }
    }

    void stochastic_scheduling_mono_10_20_10()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_20tasks_10robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 20, 10> allocation;
        allocation << 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        MonolithicStochasticMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule        = scheduler.solve();
        const float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        const float s_time   = smp_time - mp_time;
        fmt::print("MP Time: {0:f}s\nS Time: {1:f}s\nMakespan: {2:f}\nNum MILPs run: {3:d}\n",
                   mp_time,
                   s_time,
                   schedule->schedule()->makespan(),
                   MilpSchedulerBase::numIterations());
    }

    void stochastic_scheduling_bender_10_20_10()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_20tasks_10robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        Eigen::Matrix<float, 20, 10> allocation;
        allocation << 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f;

        auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

        BendersStochasticMilpScheduler scheduler(scheduler_problem_inputs);
        auto schedule        = scheduler.solve();
        const float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
        const float s_time   = smp_time - mp_time;
        fmt::print("MP Time: {0:f}s\nS Time: {1:f}s\nMakespan: {2:f}\nNum MILPs run: {3:d}\n",
                   mp_time,
                   s_time,
                   schedule->schedule()->makespan(),
                   MilpSchedulerBase::numIterations());
    }

    void stochastic_itags_10_10_5()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_10tasks_5robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        ItagsBuilderOptions builder_options = {.scheduler =
                                                   ItagsBuilderOptions::SchedulerOptions::e_monolithic_stochastic_milp};
        ItagsBuilder builder(builder_options);
        std::shared_ptr<Itags> itags = builder.build(itags_problem_inputs);
        auto results                 = itags->search();
        results.writeToFile("output.json", itags_problem_inputs);
        const float mp_time    = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float smp_time   = TimeKeeper::instance().time(constants::k_scheduling_time);
        const float s_time     = smp_time - mp_time;
        const float total_time = TimeKeeper::instance().time("itags");
        const float ta_time    = total_time - smp_time;
        fmt::print("Total: {0:f}s\nTA: {1:f}s\nS: {2:f}\nMP: {3:f}\nSuccess: {4:d}\nMakespan: {5:f}\nNum "
                   "Milp Run: {6:d}\nNum Milp Failures: {7:d}\nNum Nodes: {8:d}",
                   total_time,
                   ta_time,
                   s_time,
                   mp_time,
                   results.foundGoal(),
                   results.foundGoal() ? results.goal()->schedule()->makespan() : -1.0f,
                   MilpSchedulerBase::numIterations() - 1,
                   MilpSchedulerBase::numFailures(),
                   results.statistics()->numberOfNodesEvaluated());
    }

    void stochastic_itags_10_20_10()
    {
        std::ifstream in("data/itags_problems/itags_polypixel_10maps_20tasks_10robots.json");
        nlohmann::json j;
        in >> j;
        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        ItagsBuilderOptions builder_options = {.scheduler =
                                                   ItagsBuilderOptions::SchedulerOptions::e_monolithic_stochastic_milp};
        ItagsBuilder builder(builder_options);
        std::shared_ptr<Itags> itags = builder.build(itags_problem_inputs);
        auto results                 = itags->search();
        const float mp_time          = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float smp_time         = TimeKeeper::instance().time(constants::k_scheduling_time);
        const float s_time           = smp_time - mp_time;
        const float total_time       = TimeKeeper::instance().time("itags");
        const float ta_time          = total_time - smp_time;
        fmt::print("Total: {0:f}s\nTA: {1:f}s\nS: {2:f}\nMP: {3:f}\nSuccess: {4:d}\nMakespan: {5:f}\nNum "
                   "Milp Run: {6:d}\nNum Milp Failures: {7:d}\nNum Nodes: {8:d}",
                   total_time,
                   ta_time,
                   s_time,
                   mp_time,
                   results.foundGoal(),
                   results.foundGoal() ? results.goal()->schedule()->makespan() : -1.0f,
                   MilpSchedulerBase::numIterations() - 1,
                   MilpSchedulerBase::numFailures(),
                   results.statistics()->numberOfNodesEvaluated());
    }

    void stochastic_itags_vary_20_10()
    {
        for(unsigned int i = 1; i <= 10; ++i)
        {
            std::ifstream in(fmt::format("data/itags_problems/itags_polypixel_{0:d}0maps_20tasks_10robots.json", i));
            nlohmann::json j;
            in >> j;
            auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

            ItagsBuilderOptions builder_options = {
                .scheduler = ItagsBuilderOptions::SchedulerOptions::e_monolithic_stochastic_milp};
            ItagsBuilder builder(builder_options);
            std::shared_ptr<Itags> itags = builder.build(itags_problem_inputs);
            auto results                 = itags->search();
            const float mp_time          = TimeKeeper::instance().time(constants::k_motion_planning_time);
            const float smp_time         = TimeKeeper::instance().time(constants::k_scheduling_time);
            const float s_time           = smp_time - mp_time;
            const float total_time       = TimeKeeper::instance().time("itags");
            const float ta_time          = total_time - smp_time;
            fmt::print("Maps: {9:d}0\n\tTotal: {0:f}s\n\tTA: {1:f}s\n\tS: {2:f}s\n\tMP: {3:f}s\n\tSuccess: "
                       "{4:d}\n\tMakespan: {5:f}\n\tNum "
                       "Milp Run: {6:d}\n\tNum Milp Failures: {7:d}\n\tNum Nodes: {8:d}\n",
                       total_time,
                       ta_time,
                       s_time,
                       mp_time,
                       results.foundGoal(),
                       results.foundGoal() ? results.goal()->schedule()->makespan() : -1.0f,
                       MilpSchedulerBase::numIterations() - 1,
                       MilpSchedulerBase::numFailures(),
                       results.statistics()->numberOfNodesEvaluated(),
                       i);
            TimeKeeper::instance().resetAll();
        }
    }

    void stochastic_schedule_vary_vary_20_10()
    {
        for(unsigned int i = 1; i <= 10; ++i)
        {
            std::ifstream in(fmt::format("data/itags_problems/itags_polypixel_{0:d}0maps_20tasks_10robots.json", i));
            nlohmann::json j;
            in >> j;
            auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();
            Eigen::Matrix<float, 20, 10> allocation;
            allocation << 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

            auto scheduler_problem_inputs = std::make_shared<SchedulerProblemInputs>(itags_problem_inputs, allocation);

            fmt::print("Scenarios: {0:d}0\n", i);
            //                        {
            //                            MonolithicStochasticMilpScheduler scheduler(scheduler_problem_inputs);
            //                            auto schedule        = scheduler.solve();
            //                            const float mp_time  =
            //                            TimeKeeper::instance().time(constants::k_motion_planning_time); const float
            //                            smp_time = TimeKeeper::instance().time(constants::k_scheduling_time); const
            //                            float s_time   = smp_time - mp_time; fmt::print("\tMono:\n\t\tTime:
            //                            {0:0.3f}s\n\t\tMakespan: {1:0.3f}\n",
            //                                       s_time,
            //                                       schedule->schedule()->makespan());
            //                            TimeKeeper::instance().resetAll();
            //                        }
            {
                BendersStochasticMilpScheduler scheduler(scheduler_problem_inputs);
                auto schedule        = scheduler.solve();
                const float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
                const float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
                const float s_time   = smp_time - mp_time;
                fmt::print("\tBender:\n\t\tTime: {0:0.3f}s\n\t\tMakespan: {1:0.3f}\n",
                           s_time,
                           schedule->schedule()->makespan());
                TimeKeeper::instance().resetAll();
            }
            //            {
            //                BendersParallelStochasticMilpScheduler scheduler(scheduler_problem_inputs);
            //                auto schedule        = scheduler.solve();
            //                const float mp_time  = TimeKeeper::instance().time(constants::k_motion_planning_time);
            //                const float smp_time = TimeKeeper::instance().time(constants::k_scheduling_time);
            //                const float s_time   = smp_time - mp_time;
            //                fmt::print("\tParallel:\n\t\tTime: {0:0.3f}s\n\t\tMakespan: {1:0.3f}\n",
            //                           s_time,
            //                           schedule->schedule()->makespan());
            //                TimeKeeper::instance().resetAll();
            //            }
            return;
        }
    }

    void cdeb()
    {
        std::ifstream in("data/itags_problems/cdeb_input.json");
        nlohmann::json j;
        in >> j;

        auto itags_problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        ItagsBuilderOptions builder_options{
            .prepruning = {ItagsBuilderOptions::PrepruningMethodOptions::e_no_trait_improvement,
                           ItagsBuilderOptions::PrepruningMethodOptions::e_previous_failure_reason}};
        ItagsBuilder builder(builder_options);
        std::shared_ptr<Itags> itags = builder.build(itags_problem_inputs);

        auto result = itags->search();
        //        float mp_time    = TimeKeeper::instance().time(constants::k_motion_planning_time);
        //        float smp_time   = TimeKeeper::instance().time(constants::k_scheduling_time);
        //        float s_time     = smp_time - mp_time;
        //        float total_time = TimeKeeper::instance().time("itags");
        //        float ta_time    = total_time - smp_time;
        //        fmt::print("MP Time: {0:f}s\nS Time: {1:f}s\nTA Time: {2:f}\nTotal Time: {3:f}\nNum MILPs run:
        //        {4:d}\n",
        //                   mp_time,
        //                   s_time,
        //                   ta_time,
        //                   total_time,
        //                   MilpSchedulerBase::numIterations());
        result.writeToFile("cdeb_output.json", itags_problem_inputs);
    }

    int main(int argc, char** argv)
    {
        stochastic_scheduling_heuristic_10_10_5();
        return 0;
    }
}  // namespace grstapse::main

int main(int argc, char** argv)
{
    grstapse::main::main(argc, argv);
}