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
#include "grstapse/parameters/motion_planner_parameters_factory.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    MotionPlannerParametersFactory& MotionPlannerParametersFactory::instance()
    {
        static MotionPlannerParametersFactory singleton;
        return singleton;
    }

    MotionPlannerParametersFactory::MotionPlannerParametersFactory()
        : AlgorithmParametersFactoryBase(constants::k_motion_planner_parameters)
    {
        setParent(constants::k_euclidean_graph_motion_planner_parameters, constants::k_motion_planner_parameters);
        setParent(constants::k_ompl_motion_planner_parameters, constants::k_motion_planner_parameters);

        setRequired(constants::k_motion_planner_parameters,
                    {{constants::k_timeout, nlohmann::json::value_t::number_float}});
        setRequired(constants::k_ompl_motion_planner_parameters,
                    {{grstapse::constants::k_timeout, nlohmann::json::value_t::number_float},
                     {grstapse::constants::k_simplify_path, nlohmann::json::value_t::boolean},
                     {grstapse::constants::k_simplify_path_timeout, nlohmann::json::value_t::number_float},
                     {grstapse::constants::k_ompl_mp_algorithm, nlohmann::json::value_t::string}});
        setRequired(constants::k_euclidean_graph_motion_planner_parameters,
                    {{constants::k_is_complete, nlohmann::json::value_t::boolean}});

        setOptional(constants::k_motion_planner_parameters, {});
        setOptional(constants::k_ompl_motion_planner_parameters,
                    {{grstapse::constants::k_solutions_window, nlohmann::json::value_t::number_unsigned},
                     {grstapse::constants::k_convergence_epsilon, nlohmann::json::value_t::number_float}});
        setOptional(constants::k_euclidean_graph_motion_planner_parameters, {});

        setDefault(constants::k_motion_planner_parameters, {});
        setDefault(constants::k_ompl_motion_planner_parameters,
                   {{grstapse::constants::k_solutions_window, 10}, {grstapse::constants::k_convergence_epsilon, 0.1f}});
        setDefault(constants::k_euclidean_graph_motion_planner_parameters, {});
    }
}  // namespace grstapse