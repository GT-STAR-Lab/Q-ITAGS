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
#include <magic_enum/magic_enum.hpp>
#include <pybind11/pybind11.h>
// Project
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/problem_inputs/itags_problem_inputs.hpp>
#include <grstapse/problem_inputs/scheduler_problem_inputs.hpp>
#include <grstapse/scheduling/milp/deterministic/deterministic_milp_subscheduler.hpp>
#include <grstapse/scheduling/milp/stochastic/monolithic/monolithic_stochastic_milp_scheduler.hpp>
#include <grstapse/scheduling/schedule_base.hpp>
#include <grstapse/task_allocation/itags/itags.hpp>
#include <grstapse/task_allocation/itags/itags_builder.hpp>
#include <grstapse/task_allocation/itags/itags_builder_options.hpp>
#include <grstapse/task_allocation/itags/itags_command_line_parser.hpp>

#define CREATE_ENUM(Module, EnumName)                                                                                  \
    {                                                                                                                  \
        pybind11::enum_<grstapse::ItagsBuilderOptions::EnumName> options(Module, #EnumName);                           \
        for(auto [enumerator, name]: magic_enum::enum_entries<grstapse::ItagsBuilderOptions::EnumName>())              \
        {                                                                                                              \
            options.value(name.data(), enumerator);                                                                    \
        }                                                                                                              \
    }

int itags_main()
{
    pybind11::object sys    = pybind11::module::import("sys");
    pybind11::list sys_argv = sys.attr("argv");
    int argc                = sys_argv.size();
    std::vector<char*> cstrings;
    cstrings.reserve(argc);
    for(auto iter: sys_argv)
    {
        cstrings.push_back(const_cast<char*>(iter.cast<std::string>().c_str()));
    }

    grstapse::ItagsCommandLineParser clp;
    return clp.main(argc, cstrings.data());
}

PYBIND11_MODULE(grstapse, module)
{
    module.doc() = "Graphically Recursive Simultaneous Task Allocation, Planning, Scheduling, and Execution";

    // region itags
    pybind11::module itags_module = module.def_submodule("itags", "Incremental Task Allocation Graph Search");
    {
        // main
        itags_module.def("main", &itags_main);

        // ItagsBuilderOptions
        pybind11::class_<grstapse::ItagsBuilderOptions> itags_builder_options(itags_module, "ItagsBuilderOptions");
        itags_builder_options.def(pybind11::init<>())
            .def_readwrite("scheduler", &grstapse::ItagsBuilderOptions::scheduler)
            .def_readwrite("heuristic", &grstapse::ItagsBuilderOptions::heuristic)
            .def_readwrite("alpha", &grstapse::ItagsBuilderOptions::alpha)
            .def_readwrite("goal_check", &grstapse::ItagsBuilderOptions::goal_check)
            .def_readwrite("successor_generator", &grstapse::ItagsBuilderOptions::successor_generator)
            .def_readwrite("memoization", &grstapse::ItagsBuilderOptions::memoization)
            .def_readwrite("prepruning", &grstapse::ItagsBuilderOptions::prepruning)
            .def_readwrite("postpruning", &grstapse::ItagsBuilderOptions::postpruning)
            .def_readwrite("use_reverse", &grstapse::ItagsBuilderOptions::use_reverse);
        CREATE_ENUM(itags_builder_options, SchedulerOptions)
        CREATE_ENUM(itags_builder_options, HeuristicOptions)
        CREATE_ENUM(itags_builder_options, GoalCheckOptions)
        CREATE_ENUM(itags_builder_options, SuccessorGeneratorOptions)
        CREATE_ENUM(itags_builder_options, MemoizationOptions)
        CREATE_ENUM(itags_builder_options, PrepruningMethodOptions)
        CREATE_ENUM(itags_builder_options, PostpruningMethodOptions)

        // ItagsBuilder
        pybind11::class_<grstapse::ItagsBuilder> builder(itags_module, "ItagsBuilder");
        builder.def(pybind11::init<grstapse::ItagsBuilderOptions>()).def("build", &grstapse::ItagsBuilder::build);

        // Itags
        // Note: Don't expose the constructor, so we can force using the builder options (much less interface overhead)
        pybind11::class_<grstapse::Itags, std::shared_ptr<grstapse::Itags>> itags_object(itags_module, "Itags");
        itags_object.def("search", &grstapse::Itags::search);

        // ProblemInputs (Needed for down-casting for SearchResults::writeToFile)
        pybind11::class_<grstapse::ProblemInputs, std::shared_ptr<grstapse::ProblemInputs>> problem_inputs(
            itags_module,
            "ProblemInputs");

        // ItagsProblemInputs
        pybind11::
            class_<grstapse::ItagsProblemInputs, grstapse::ProblemInputs, std::shared_ptr<grstapse::ItagsProblemInputs>>
                itags_problem_inputs(itags_module, "ItagsProblemInputs");
        itags_module.def("loadProblemInputsFromFile",
                         &grstapse::json_ext::loadJsonFromFile<std::shared_ptr<grstapse::ItagsProblemInputs>>);

        // SearchResults
        using ItagsResults =
            grstapse::SearchResults<grstapse::IncrementalTaskAllocationNode, grstapse::ItagsStatistics>;
        pybind11::class_<ItagsResults> itags_results(itags_module, "ItagsResults");
        itags_results.def("writeToFile", &ItagsResults::writeToFile);
    }
    // endregion
    pybind11::module scheduling_module = module.def_submodule("scheduling", "Scheduling");
    // region scheduling
    {
        // SchedulingProblemInputs
        pybind11::class_<grstapse::SchedulerProblemInputs, std::shared_ptr<grstapse::SchedulerProblemInputs>>
            scheduler_problem_inputs(scheduling_module, "SchedulerProblemInputs");
        scheduler_problem_inputs.def("precedenceConstraints", &grstapse::SchedulerProblemInputs::precedenceConstraints)
            .def("mutexConstraints", &grstapse::SchedulerProblemInputs::mutexConstraints)
            .def("numberOfPlanTasks", &grstapse::SchedulerProblemInputs::numberOfPlanTasks);
        scheduling_module.def("loadProblemInputsFromFile",
                              &grstapse::json_ext::loadJsonFromFile<std::shared_ptr<grstapse::SchedulerProblemInputs>>);

        // Deterministic Scheduler
        pybind11::class_<grstapse::DeterministicMilpScheduler> scheduler(scheduling_module,
                                                                         "DeterministicMilpScheduler");
        scheduler.def(pybind11::init<std::shared_ptr<grstapse::SchedulerProblemInputs>>())
            .def("solve", &grstapse::DeterministicMilpScheduler::solve);

        // SchedulerResult
        pybind11::class_<grstapse::SchedulerResult, std::shared_ptr<grstapse::SchedulerResult>> scheduler_result(
            scheduling_module,
            "SchedulerResult");
        scheduler_result.def("success", &grstapse::SchedulerResult::success)
            .def("failed", &grstapse::SchedulerResult::failed)
            .def("scheduler", &grstapse::SchedulerResult::schedule);

        // ScheduleBase
        pybind11::class_<grstapse::ScheduleBase, std::shared_ptr<grstapse::ScheduleBase>> schedule_base(
            scheduling_module,
            "ScheduleBase");
        schedule_base.def("makespan", &grstapse::ScheduleBase::makespan)
            .def("precedenceSetMutexConstraints", &grstapse::ScheduleBase::precedenceSetMutexConstraints);
    }
    // endregion
}