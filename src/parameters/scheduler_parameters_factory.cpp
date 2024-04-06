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
#include "grstapse/parameters/scheduler_parameters_factory.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    SchedulerParametersFactory& SchedulerParametersFactory::instance()
    {
        static SchedulerParametersFactory singleton;
        return singleton;
    }

    SchedulerParametersFactory::SchedulerParametersFactory()
        : AlgorithmParametersFactoryBase(constants::k_scheduler_parameters)
    {
        // Create parent tree
        setParent(constants::k_milp_scheduler_parameters, constants::k_scheduler_parameters);
        setParent(constants::k_deterministic_milp_scheduler_parameters, constants::k_milp_scheduler_parameters);
        setParent(constants::k_stochastic_milp_scheduler_parameters, constants::k_milp_scheduler_parameters);
        setParent(constants::k_heuristic_approximation_stochastic_scheduler_parameters,
                  constants::k_stochastic_milp_scheduler_parameters);
        setParent(constants::k_gnn_heuristic_approximation_stochastic_scheduler_parameters,
                  constants::k_heuristic_approximation_stochastic_scheduler_parameters);

        // Set required parameters
        setRequired(constants::k_scheduler_parameters, {{constants::k_timeout, nlohmann::json::value_t::number_float}});
        setRequired(constants::k_milp_scheduler_parameters,
                    {{constants::k_milp_timeout, nlohmann::json::value_t::number_float}});
        setRequired(constants::k_deterministic_milp_scheduler_parameters, {});
        setRequired(constants::k_stochastic_milp_scheduler_parameters,
                    {{constants::k_gamma, nlohmann::json::value_t::number_float},
                     {constants::k_num_scenarios, nlohmann::json::value_t::number_unsigned},
                     {constants::k_use_sprt, nlohmann::json::value_t::boolean},
                     {constants::k_delta_percentage, nlohmann::json::value_t::boolean},
                     {constants::k_delta, nlohmann::json::value_t::number_float},
                     {constants::k_indifference_tolerance, nlohmann::json::value_t::number_float}});
        setRequired(constants::k_heuristic_approximation_stochastic_scheduler_parameters,
                    {{constants::k_beta, nlohmann::json::value_t::number_unsigned}});
        setRequired(constants::k_gnn_heuristic_approximation_stochastic_scheduler_parameters,
                    {{constants::k_model_filepath, nlohmann::json::value_t::string},
                     {constants::k_model_parameters_filepath, nlohmann::json::value_t::string}});

        // Set optional parameters
        setOptional(constants::k_scheduler_parameters, {});
        setOptional(constants::k_milp_scheduler_parameters,
                    {{constants::k_threads, nlohmann::json::value_t::number_unsigned},
                     {constants::k_mip_gap, nlohmann::json::value_t::number_float},
                     {constants::k_heuristic_time, nlohmann::json::value_t::number_float},
                     {constants::k_method, nlohmann::json::value_t::number_integer},
                     {constants::k_return_feasible_on_timeout, nlohmann::json::value_t::boolean}});
        setOptional(constants::k_deterministic_milp_scheduler_parameters,
                    {{constants::k_use_hierarchical_objective, nlohmann::json::value_t::boolean}});
        setOptional(constants::k_stochastic_milp_scheduler_parameters, {});
        setOptional(constants::k_heuristic_approximation_stochastic_scheduler_parameters, {});
        setOptional(constants::k_gnn_heuristic_approximation_stochastic_scheduler_parameters, {});

        // Set default values for optional parameters
        setDefault(constants::k_scheduler_parameters, {});
        setDefault(constants::k_milp_scheduler_parameters,
                   {{constants::k_threads, 0},
                    {constants::k_mip_gap, -1.0f},
                    {constants::k_heuristic_time, -1.0f},
                    {constants::k_method, -1},
                    {constants::k_return_feasible_on_timeout, false}});
        setDefault(constants::k_deterministic_milp_scheduler_parameters,
                   {{constants::k_use_hierarchical_objective, false}});
        setDefault(constants::k_stochastic_milp_scheduler_parameters, {});
        setDefault(constants::k_heuristic_approximation_stochastic_scheduler_parameters, {});
        setDefault(constants::k_gnn_heuristic_approximation_stochastic_scheduler_parameters, {});
    }
}  // namespace grstapse