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
#include "grstapse/task_allocation/itags/itags_command_line_parser.hpp"

// External
#include <cli11/cli11.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/cli11_extension.hpp"
#include "grstapse/config.hpp"
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/itags.hpp"
#include "grstapse/task_allocation/itags/itags_builder.hpp"

namespace grstapse
{
    int ItagsCommandLineParser::main(int argc, char** argv)
    {
        // TODO(Andrew): print version and license?
        CLI::App app{"Incremental Task Allocation Graph Search", "itags"};
        app.formatter(std::make_shared<cli11_ext::CustomCliFormatter>());

        app.set_version_flag("-v,--version", fmt::format("{0:s}\ngit hash: {1:s}", s_project_version, s_git_hash));
        addInputArguments(app);
        addOutputArguments(app);

        CLI::Option* config = app.add_option("config-file",
                                             m_json_config_filepath,
                                             "The filepath to the json file for building a custom ITAGS configuration")
                                  ->check(CLI::ExistingFile.description(""));

        CLI::App* config_app = app.add_option_group("config")->excludes(config);

        // Setup Command Line Arguments
        addSchedulerArguments(*config_app);
        addHeuristicArguments(*config_app);
        addGoalCheckArguments(*config_app);
        addSuccessorGeneratorArguments(*config_app);
        addMemoizationArguments(*config_app);
        addPrepruningArguments(*config_app);
        addPostpruningArguments(*config_app);

        try
        {
            app.parse(argc, argv);
        }
        catch(const CLI::ParseError& e)
        {
            return app.exit(e);
        }

        // Load problem inputs
        std::ifstream fin(m_problem_input_filepath);
        nlohmann::json j;
        fin >> j;
        auto problem_inputs = j.get<std::shared_ptr<ItagsProblemInputs>>();

        // Build ITAGS, run, and write output
        ItagsBuilder builder(m_builder_options);
        std::shared_ptr<Itags> itags = builder.build(problem_inputs);
        auto results                 = itags->search();
        results.writeToFile(m_solution_output_filepath, problem_inputs);

        return 0;
    }

    void ItagsCommandLineParser::addOutputArguments(CLI::App& app)
    {
        app.add_option("output", m_solution_output_filepath, "The filepath to where the output json should be created")
            ->required()
            ->check(CLI::NonexistentPath.description(""));
    }

    void ItagsCommandLineParser::addInputArguments(CLI::App& app)
    {
        app.add_option("problem-input", m_problem_input_filepath, "The filepath to the problem inputs json")
            ->required()
            ->check(CLI::ExistingFile.description(""));
    }

    void ItagsCommandLineParser::addSchedulerArguments(CLI::App& app)
    {
        cli11_ext::createEnumArgument<ItagsBuilderOptions::SchedulerOptions>(
            app,
            {m_builder_options.scheduler,
             "Scheduler",
             "The scheduling algorithm to use",
             {{ItagsBuilderOptions::SchedulerOptions::e_deterministic_milp, "Deterministic MILP"},
              {ItagsBuilderOptions::SchedulerOptions::e_monolithic_stochastic_milp, "Monolithic Stochastic MILP"},
              {ItagsBuilderOptions::SchedulerOptions::e_benders_stochastic_milp, "Benders Stochastic MILP"},
              {ItagsBuilderOptions::SchedulerOptions::e_benders_parallel_stochastic_milp,
               "Benders Parallel Stochastic MILP"}}});
    }

    void ItagsCommandLineParser::addHeuristicArguments(CLI::App& app)
    {
        cli11_ext::CliEnumParameters parameters{
            m_builder_options.heuristic,
            "Heuristic",
            "The algorithm to use to estimate a node's distance to the goal",
            {{ItagsBuilderOptions::HeuristicOptions::e_tetaq, "Time-Extended Task Allocation Quality (TETAQ)"},
             {ItagsBuilderOptions::HeuristicOptions::e_apr, "Allocation Percentage Remaining (APR)"},
             {ItagsBuilderOptions::HeuristicOptions::e_nsq, "Normalized Schedule Quality (NSQ)"}},
        };
        parameters.addExtraArgument(
            ItagsBuilderOptions::HeuristicOptions::e_tetaq,
            {"--alpha", m_builder_options.alpha, "The alpha parameter for TETAQ's linear combination"});
        cli11_ext::createEnumArgument(app, std::move(parameters));
    }

    void ItagsCommandLineParser::addGoalCheckArguments(CLI::App& app)
    {
        cli11_ext::createEnumArgument<ItagsBuilderOptions::GoalCheckOptions>(
            app,
            {m_builder_options.goal_check,
             "Goal Check",
             "The algorithm to use to check if a search node is the goal",
             {{ItagsBuilderOptions::GoalCheckOptions::e_zero_apr,
               "A node is considered the goal when its APR value is 0"}}});
    }

    void ItagsCommandLineParser::addSuccessorGeneratorArguments(CLI::App& app)
    {
        cli11_ext::createEnumArgument<ItagsBuilderOptions::SuccessorGeneratorOptions>(
            app,
            {m_builder_options.successor_generator,
             "Successor Generator",
             "The algorithm to use to generate the successors of node",
             {{ItagsBuilderOptions::SuccessorGeneratorOptions::e_increment,
               "Adds a single robot to a single task (all possible combinations)"}}});
    }

    void ItagsCommandLineParser::addMemoizationArguments(CLI::App& app)
    {
        cli11_ext::createEnumArgument<ItagsBuilderOptions::MemoizationOptions>(
            app,
            {m_builder_options.memoization,
             "Memoization",
             "The algorithm to use to check if a node is the same as one that has already been visited",
             {{ItagsBuilderOptions::MemoizationOptions::e_null, "Each node is considered different"},
              {ItagsBuilderOptions::MemoizationOptions::e_hash, "Hashes based on the allocation matrix"}}});
    }

    void ItagsCommandLineParser::addPrepruningArguments(CLI::App& app)
    {
        cli11_ext::createEnumListArgument<ItagsBuilderOptions::PrepruningMethodOptions>(
            app,
            {m_builder_options.prepruning,
             "Prepruning",
             "The pruning methods to use before the heuristic is evaluated",
             {{ItagsBuilderOptions::PrepruningMethodOptions::e_null, "No pruning method is used"},
              {ItagsBuilderOptions::PrepruningMethodOptions::e_no_trait_improvement,
               "Prunes a node if it does not decrease the APR value from its parent"},
              {ItagsBuilderOptions::PrepruningMethodOptions::e_previous_failure_reason,
               "Prunes a node if it allocation a robot to a task that has been deemed impossible "
               "previously by either scheduling or motion planning"}}});
    }

    void ItagsCommandLineParser::addPostpruningArguments(CLI::App& app)
    {
        cli11_ext::createEnumListArgument<ItagsBuilderOptions::PostpruningMethodOptions>(
            app,
            {m_builder_options.postpruning,
             "Postpruning",
             "The pruning methods to use after the heuristic is evaluated",
             {{ItagsBuilderOptions::PostpruningMethodOptions::e_null, "No pruning method is used"}}});
    }
}  // namespace grstapse