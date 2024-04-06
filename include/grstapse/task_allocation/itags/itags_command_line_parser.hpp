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
#pragma once

// Global
#include <string>
// Local
#include "grstapse/task_allocation/itags/itags_builder_options.hpp"

namespace CLI
{
    class App;
    class Option;
}  // namespace CLI

namespace grstapse
{
    /*!
     * \brief Interface for command line parsing for itags
     */
    class ItagsCommandLineParser
    {
       public:
        int main(int argc, char** argv);

       protected:
        void addInputArguments(CLI::App& app);
        void addOutputArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the scheduler to use
        void addSchedulerArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the heuristic to use
        void addHeuristicArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the successor generator to use
        void addSuccessorGeneratorArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the goal check to use
        void addGoalCheckArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the memoization to use
        void addMemoizationArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the prepruning to use
        void addPrepruningArguments(CLI::App& app);
        //! Adds the command line arguments for selecting the postpruning to use
        void addPostpruningArguments(CLI::App& app);

        std::string m_json_config_filepath;
        std::string m_problem_input_filepath;
        std::string m_solution_output_filepath;
        ItagsBuilderOptions m_builder_options;
    };

}  // namespace grstapse