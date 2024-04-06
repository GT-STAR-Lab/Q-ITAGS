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
#include <string_view>

/*!
 * \macro CREATE_CONSTANT
 *
 * \brief Creates a string_view constant
 */
#define CREATE_JSON_KEY(name) constexpr const char* const k_##name = #name;

namespace grstapse::constants
{
    // region Json Keys
    CREATE_JSON_KEY(algorithm_parameters)
    CREATE_JSON_KEY(allocation)
    CREATE_JSON_KEY(alpha)
    CREATE_JSON_KEY(best_schedule)
    CREATE_JSON_KEY(beta)
    CREATE_JSON_KEY(bounding_radius)
    CREATE_JSON_KEY(bounds)
    CREATE_JSON_KEY(coalition)
    CREATE_JSON_KEY(config_type)
    CREATE_JSON_KEY(configuration_type)
    CREATE_JSON_KEY(constraint_tree_node_cost_type)
    CREATE_JSON_KEY(convergence_epsilon)
    CREATE_JSON_KEY(cost)
    CREATE_JSON_KEY(delta)
    CREATE_JSON_KEY(delta_percentage)
    CREATE_JSON_KEY(desired_traits)
    CREATE_JSON_KEY(domain_filepath)
    CREATE_JSON_KEY(dubins)
    CREATE_JSON_KEY(duration)
    CREATE_JSON_KEY(edges)
    CREATE_JSON_KEY(environment_parameters)
    CREATE_JSON_KEY(execution_motion_plan)
    CREATE_JSON_KEY(fcpop_parameters)
    CREATE_JSON_KEY(finish_timepoint)
    CREATE_JSON_KEY(gamma)
    CREATE_JSON_KEY(goal_type)
    CREATE_JSON_KEY(graph_filepath)
    CREATE_JSON_KEY(graph_type)
    CREATE_JSON_KEY(has_timeout)
    CREATE_JSON_KEY(heuristic_time)
    CREATE_JSON_KEY(high)
    CREATE_JSON_KEY(id)
    CREATE_JSON_KEY(image)
    CREATE_JSON_KEY(indifference_tolerance)
    CREATE_JSON_KEY(individual_plan)
    CREATE_JSON_KEY(initial_configuration)
    CREATE_JSON_KEY(is_complete)
    CREATE_JSON_KEY(itags_parameters)
    CREATE_JSON_KEY(last_edge)
    CREATE_JSON_KEY(linear_quality_coefficients)
    CREATE_JSON_KEY(low)
    CREATE_JSON_KEY(low_level_timer_name)
    CREATE_JSON_KEY(makespan)
    CREATE_JSON_KEY(masked)
    CREATE_JSON_KEY(max_schedule)
    CREATE_JSON_KEY(method)
    CREATE_JSON_KEY(milp_scheduler_type)
    CREATE_JSON_KEY(milp_timeout)
    CREATE_JSON_KEY(mip_gap)
    CREATE_JSON_KEY(model_filepath)
    CREATE_JSON_KEY(model_parameters_filepath)
    CREATE_JSON_KEY(motion_planner_parameters)
    CREATE_JSON_KEY(motion_planner_type)
    CREATE_JSON_KEY(motion_planners)
    CREATE_JSON_KEY(motion_planning_time)
    CREATE_JSON_KEY(mp_index)
    CREATE_JSON_KEY(mp_parameters)
    CREATE_JSON_KEY(mp_type)
    CREATE_JSON_KEY(mutex_constraints)
    CREATE_JSON_KEY(name)
    CREATE_JSON_KEY(nodes_deadend)
    CREATE_JSON_KEY(nodes_evaluated)
    CREATE_JSON_KEY(nodes_expanded)
    CREATE_JSON_KEY(nodes_generated)
    CREATE_JSON_KEY(nodes_pruned)
    CREATE_JSON_KEY(nodes_reopened)
    CREATE_JSON_KEY(num_motion_plan_failures)
    CREATE_JSON_KEY(num_motion_plans)
    CREATE_JSON_KEY(num_scenarios)
    CREATE_JSON_KEY(num_scheduling_failures)
    CREATE_JSON_KEY(num_scheduling_iterations)
    CREATE_JSON_KEY(ompl_environment_type)
    CREATE_JSON_KEY(ompl_mp_algorithm)
    CREATE_JSON_KEY(origin)
    CREATE_JSON_KEY(parallel_sprt)
    CREATE_JSON_KEY(path)
    CREATE_JSON_KEY(path_cost_time)
    CREATE_JSON_KEY(path_length)
    CREATE_JSON_KEY(pddl)
    CREATE_JSON_KEY(pgm_filepath)
    CREATE_JSON_KEY(plan_task_indices)
    CREATE_JSON_KEY(point_graph_type)
    CREATE_JSON_KEY(precedence_constraints)
    CREATE_JSON_KEY(precedence_set_mutex_constraints)
    CREATE_JSON_KEY(problem_filepath)
    CREATE_JSON_KEY(qw)
    CREATE_JSON_KEY(qx)
    CREATE_JSON_KEY(qy)
    CREATE_JSON_KEY(qz)
    CREATE_JSON_KEY(rebuild)
    CREATE_JSON_KEY(resolution)
    CREATE_JSON_KEY(return_feasible_on_timeout)
    CREATE_JSON_KEY(robot_traits_matrix_reduction)
    CREATE_JSON_KEY(robots)
    CREATE_JSON_KEY(rotation)
    CREATE_JSON_KEY(save_closed_nodes)
    CREATE_JSON_KEY(save_pruned_nodes)
    CREATE_JSON_KEY(scheduler_parameters)
    CREATE_JSON_KEY(scheduler_type)
    CREATE_JSON_KEY(scheduling_time)
    CREATE_JSON_KEY(search_parameters)
    CREATE_JSON_KEY(simplify_path)
    CREATE_JSON_KEY(simplify_path_timeout)
    CREATE_JSON_KEY(solution)
    CREATE_JSON_KEY(solutions_window)
    CREATE_JSON_KEY(species)
    CREATE_JSON_KEY(speed)
    CREATE_JSON_KEY(start_timepoint)
    CREATE_JSON_KEY(state_space_type)
    CREATE_JSON_KEY(state_type)
    CREATE_JSON_KEY(states)
    CREATE_JSON_KEY(statistics)
    CREATE_JSON_KEY(task_allocation_time)
    CREATE_JSON_KEY(task_associations)
    CREATE_JSON_KEY(task_planning_time)
    CREATE_JSON_KEY(tasks)
    CREATE_JSON_KEY(terminal_configuration)
    CREATE_JSON_KEY(threads)
    CREATE_JSON_KEY(threshold)
    CREATE_JSON_KEY(time)
    CREATE_JSON_KEY(timeout)
    CREATE_JSON_KEY(timer_name)
    CREATE_JSON_KEY(total_time)
    CREATE_JSON_KEY(traits)
    CREATE_JSON_KEY(transitions)
    CREATE_JSON_KEY(turning_radius)
    CREATE_JSON_KEY(use_data_dir)
    CREATE_JSON_KEY(use_hierarchical_objective)
    CREATE_JSON_KEY(use_reverse)
    CREATE_JSON_KEY(use_sprt)
    CREATE_JSON_KEY(vector_reduction_function_type)
    CREATE_JSON_KEY(vertex)
    CREATE_JSON_KEY(vertex_a)
    CREATE_JSON_KEY(vertex_b)
    CREATE_JSON_KEY(vertices)
    CREATE_JSON_KEY(w)
    CREATE_JSON_KEY(worst_schedule)
    CREATE_JSON_KEY(x)
    CREATE_JSON_KEY(y)
    CREATE_JSON_KEY(yaml_filepath)
    CREATE_JSON_KEY(yaw)
    CREATE_JSON_KEY(z)
    // endregion

    // region Search Parameter Types
    constexpr std::string_view k_best_first_search_parameters     = "BestFirstSearchParameters";
    constexpr std::string_view k_focal_a_star_parameters          = "FocalAStarParameters";
    constexpr std::string_view k_conflict_based_search_parameters = "ConflictBasedSearchParameters";
    // endregion

    // region Motion Planner Parameter Types
    constexpr std::string_view k_euclidean_graph_motion_planner_parameters = "EuclideanGraphMotionPlannerParameters";
    constexpr std::string_view k_ompl_motion_planner_parameters            = "OmplMotionPlannerParameters";
    // endregion

    // region Motion Planner Types
    constexpr std::string_view k_complete_euclidean_graph_motion_planner = "CompleteEuclideanGraphMotionPlanner";
    constexpr std::string_view k_complete_sampled_euclidean_graph_motion_planner =
        "CompleteSampledEuclideanGraphMotionPlanner";
    constexpr std::string_view k_euclidean_graph_motion_planner = "EuclideanGraphMotionPlanner";
    constexpr std::string_view k_masked_complete_sampled_euclidean_graph_motion_planner =
        "MaskedCompleteSampledEuclideanGraphMotionPlanner";
    constexpr std::string_view k_ompl_motion_planner                    = "OmplMotionPlanner";
    constexpr std::string_view k_sampled_euclidean_graph_motion_planner = "SampledEuclideanGraphMotionPlanner";

    // endregion

    // region Environment Types
    constexpr std::string_view k_pgm_ompl_environment = "PgmOmplEnvironment";
    // endregion

    // region Scheduler Parameter Types
    constexpr std::string_view k_deterministic_milp_scheduler_parameters = "DeterministicMilpSchedulerParameters";
    constexpr std::string_view k_heuristic_approximation_stochastic_scheduler_parameters =
        "HeuristicApproximationStochasticSchedulerParameters";
    constexpr std::string_view k_milp_scheduler_parameters            = "MilpSchedulerParameters";
    constexpr std::string_view k_stochastic_milp_scheduler_parameters = "StochasticMilpSchedulerParameters";
    constexpr std::string_view k_gnn_heuristic_approximation_stochastic_scheduler_parameters =
        "GnnHeuristicApproximationStochasticSchedulerParameters";
    // endregion
}  // namespace grstapse::constants