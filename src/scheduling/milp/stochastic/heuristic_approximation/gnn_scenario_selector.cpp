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
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/gnn_scenario_selector.hpp"

// External
#include <pybind11/stl.h>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/take.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/config.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_tasks_info.hpp"
#include "grstapse/scheduling/milp/deterministic/dms_all_transitions_info.hpp"
#include "grstapse/scheduling/milp/deterministic/subscheduler_motion_planner_interface.hpp"
#include "grstapse/scheduling/milp/mutex_indicators.hpp"

namespace py = pybind11;
using namespace py::literals;

namespace grstapse
{
    GnnScenarioSelector::GnnScenarioSelector(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
                                             bool use_cpu,
                                             unsigned int gpu_id)
        : ScenarioSelectorBase(problem_inputs)
        , m_use_cpu(use_cpu)
        , m_gpu_id(gpu_id)
    {
        auto parameters                      = problem_inputs->schedulerParameters();
        std::string model_filepath           = parameters->get<std::string>(constants::k_model_filepath);
        std::string model_parameter_filepath = parameters->get<std::string>(constants::k_model_parameters_filepath);
        loadModel(model_filepath, model_parameter_filepath);
    }

    //! Convert std::set<std::pair<unsigned int, unsigned int>> to std::vector<std::pair<int, int>>
    std::vector<std::pair<int, int>> setToVector(const std::set<std::pair<unsigned int, unsigned int>>& s)
    {
        return s |
               ::ranges::view::transform(
                   [](const std::pair<unsigned int, unsigned int>& p) -> std::pair<int, int>
                   {
                       return {p.first, p.second};
                   }) |
               ::ranges::to<std::vector>();
    }

    //! Convert std::set<std::pair<unsigned int, unsigned int>> to std::vector<std::pair<int, int>> and filter based on
    //! \p filter
    std::vector<std::pair<int, int>> setToVector(const std::set<std::pair<unsigned int, unsigned int>>& s, auto filter)
    {
        return s | ::ranges::view::filter(filter) |
               ::ranges::view::transform(
                   [](const std::pair<unsigned int, unsigned int>& p) -> std::pair<int, int>
                   {
                       return {p.first, p.second};
                   }) |
               ::ranges::to<std::vector>();
    }

    std::optional<std::vector<bool>> GnnScenarioSelector::createMask(
        Timer& timer,
        const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
        unsigned int num_samples,
        unsigned int beta,
        float gamma,
        float timeout)
    {
        py::list predictions = m_module.attr("get_predictions_graph")(m_model,
                                                                      m_device,
                                                                      py::int_(num_samples),
                                                                      py::float_(gamma),
                                                                      buildGraph(motion_planner, num_samples),
                                                                      py::bool_(m_use_cpu),
                                                                      py::int_(m_gpu_id));

        std::set<std::pair<float, unsigned int>, std::greater<std::pair<float, unsigned int>>> z;
        for(unsigned int index = 0; index < num_samples; ++index)
        {
            z.insert({predictions[index].cast<float>(), index});
        }

        std::vector<bool> rv(num_samples, false);
        for(const auto& [val, index]: z | ::ranges::view::take(beta))
        {
            rv[index] = true;
        }
        return rv;
    }

    void GnnScenarioSelector::loadModel(const std::string& model_filepath, const std::string& model_parameters_filepath)
    {
        // Add the embed directory to the path to search for modules
        py::module::import("sys").attr("path").attr("append")(s_python_embed_scripts_dir);
        m_module = py::module::import("gnn_scenario_selector");

        Logger::info("Loading model: {0:s} ({1:s})", model_filepath, model_parameters_filepath);
        py::list rv = m_module.attr(
            "load_gnn")(model_filepath, model_parameters_filepath, py::bool_(m_use_cpu), py::int_(m_gpu_id));
        m_model  = std::move(rv[0]);
        m_device = std::move(rv[1]);
    }

    pybind11::dict GnnScenarioSelector::buildDataDict(
        const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
        unsigned int num_samples)
    {
        py::dict rv;
        rv["lower_bounds"] = py::dict();
        rv["precedence"]   = py::dict();
        rv["mutex"]        = py::dict();
        for(unsigned int q = 0; q < num_samples; ++q)
        {
            rv["lower_bounds"][py::int_(q)] = py::dict();
            rv["precedence"][py::int_(q)]   = py::dict();
            rv["mutex"][py::int_(q)]        = py::dict();

            auto scheduler_motion_planner_interface = std::make_shared<const SubschedulerMotionPlannerInterface>(q);
            DmsAllTasksInfo tasks_info(m_problem_inputs, nullptr, scheduler_motion_planner_interface);
            tasks_info.setupData();

            // Initial Transitions
            for(unsigned int task_nr = 0, num_tasks = m_problem_inputs->numberOfPlanTasks(); task_nr < num_tasks;
                ++task_nr)
            {
                rv["lower_bounds"][py::int_(q)][py::int_(task_nr)] = tasks_info.taskLowerBound(task_nr);
            }

            auto mutex_indicators = std::make_shared<MutexIndicators>(m_problem_inputs, nullptr);
            DmsAllTransitionsInfo transitions_info(tasks_info,
                                                   m_problem_inputs,
                                                   mutex_indicators,
                                                   nullptr,
                                                   scheduler_motion_planner_interface);
            transitions_info.setupData();

            // Precedence Constraints
            for(const auto [predecessor, successor]: m_problem_inputs->precedenceConstraints())
            {
                rv["precedence"][py::int_{q}][py::make_tuple(py::int_(predecessor), py::int_(successor))] =
                    tasks_info.taskDuration(predecessor) +
                    transitions_info.transitionDurationLowerBound(predecessor, successor);
            }

            for(const auto [i, j]: mutex_indicators->indicators() | ::ranges::view::keys)
            {
                rv["mutex"][py::int_{q}][py::make_tuple(pybind11::int_(i), pybind11::int_(j))] =
                    tasks_info.taskDuration(i) + transitions_info.transitionDurationLowerBound(i, j);
                rv["mutex"][py::int_{q}][py::make_tuple(pybind11::int_(j), pybind11::int_(i))] =
                    tasks_info.taskDuration(j) + transitions_info.transitionDurationLowerBound(j, i);
            }
        }

        return rv;
    }
    pybind11::object GnnScenarioSelector::buildGraph(
        const std::shared_ptr<MaskedCompleteSampledEuclideanGraphMotionPlanner>& motion_planner,
        unsigned int num_samples)
    {
        py::module dgl = pybind11::module::import("dgl");
        py::module th  = pybind11::module::import("torch");

        py::list graph_list;
        for(unsigned int q = 0; q < num_samples; ++q)
        {
            auto scheduler_motion_planner_interface = std::make_shared<const SubschedulerMotionPlannerInterface>(q);
            DmsAllTasksInfo tasks_info(m_problem_inputs, nullptr, scheduler_motion_planner_interface);
            tasks_info.setupData();

            py::list node_feature_list;
            for(unsigned int task_nr = 0, num_tasks = m_problem_inputs->numberOfPlanTasks(); task_nr < num_tasks;
                ++task_nr)
            {
                py::list l;
                l.append(tasks_info.taskLowerBound(task_nr));
                node_feature_list.append(std::move(l));
            }

            py::list u_list;
            py::list v_list;
            py::list edge_feature_list;
            const auto& precedence_constraints = m_problem_inputs->precedenceConstraints();
            auto mutex_indicators              = std::make_shared<MutexIndicators>(m_problem_inputs, nullptr);
            DmsAllTransitionsInfo transitions_info(tasks_info,
                                                   m_problem_inputs,
                                                   mutex_indicators,
                                                   nullptr,
                                                   scheduler_motion_planner_interface);
            transitions_info.setupData();

            for(auto [predecessor, successor]: precedence_constraints)
            {
                u_list.append(predecessor);
                v_list.append(successor);
                py::list l;
                l.append(0);
                l.append(tasks_info.taskDuration(predecessor) +
                         transitions_info.transitionDurationLowerBound(predecessor, successor));
                edge_feature_list.append(std::move(l));
            }
            for(auto [i, j]: m_problem_inputs->mutexConstraints())
            {
                if(precedence_constraints.contains({i, j}) or precedence_constraints.contains({j, i}))
                {
                    continue;
                }

                // i -> j
                u_list.append(py::int_(i));
                v_list.append(py::int_(j));
                {
                    py::list l;
                    l.append(1);
                    l.append(tasks_info.taskDuration(i) + transitions_info.transitionDurationLowerBound(i, j));
                    edge_feature_list.append(std::move(l));
                }

                // i -> j
                u_list.append(j);
                v_list.append(i);
                {
                    py::list l;
                    l.append(1);
                    l.append(tasks_info.taskDuration(j) + transitions_info.transitionDurationLowerBound(j, i));
                    edge_feature_list.append(std::move(l));
                }
            }
            // Note: By setting both tensors to use m_device the graph itself will be constructed on m_device
            py::object graph =
                dgl.attr("graph")(py::make_tuple(th.attr("tensor")(std::move(u_list), "device"_a = m_device),
                                                 th.attr("tensor")(std::move(v_list), "device"_a = m_device)),
                                  "num_nodes"_a = m_problem_inputs->numberOfPlanTasks());

            graph.attr("ndata")["feat"] = th.attr("tensor")(node_feature_list, "device"_a = m_device);
            graph.attr("edata")["feat"] = th.attr("tensor")(edge_feature_list, "device"_a = m_device);

            graph_list.append(std::move(graph));
        }
        return graph_list;
    }
}  // namespace grstapse