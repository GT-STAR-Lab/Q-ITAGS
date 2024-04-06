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
#include "grstapse/parameters/search_parameters_factory.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_base.hpp"

namespace grstapse
{
    SearchParametersFactory& SearchParametersFactory::instance()
    {
        static SearchParametersFactory singleton;
        return singleton;
    }
    SearchParametersFactory::SearchParametersFactory()
        : AlgorithmParametersFactoryBase(constants::k_search_parameters)
    {
        // Set parent tree
        setParent(constants::k_best_first_search_parameters, constants::k_search_parameters);
        setParent(constants::k_focal_a_star_parameters, constants::k_best_first_search_parameters);
        setParent(constants::k_conflict_based_search_parameters, constants::k_search_parameters);

        // Set required parameters
        setRequired(constants::k_search_parameters,
                    {{constants::k_has_timeout, nlohmann::json::value_t::boolean},
                     {constants::k_timeout, nlohmann::json::value_t::number_float},
                     {constants::k_timer_name, nlohmann::json::value_t::string}});
        setRequired(constants::k_best_first_search_parameters, {});
        setRequired(constants::k_focal_a_star_parameters,
                    {{constants::k_w, nlohmann::json::value_t::number_float},
                     {constants::k_rebuild, nlohmann::json::value_t::boolean}});
        setRequired(constants::k_conflict_based_search_parameters,
                    {{constants::k_low_level_timer_name, nlohmann::json::value_t::string}});

        // Set optional parameters
        setOptional(constants::k_search_parameters, {});
        setOptional(constants::k_best_first_search_parameters,
                    {{constants::k_save_pruned_nodes, nlohmann::json::value_t::boolean},
                     {constants::k_save_closed_nodes, nlohmann::json::value_t::boolean}});
        setOptional(constants::k_focal_a_star_parameters, {});
        setOptional(constants::k_conflict_based_search_parameters,
                    {{constants::k_constraint_tree_node_cost_type, nlohmann::json::value_t::string}});

        // Set default values for optional parameters
        setDefault(constants::k_search_parameters, {});
        setDefault(constants::k_best_first_search_parameters,
                   {{constants::k_save_pruned_nodes, false}, {constants::k_save_closed_nodes, false}});
        setDefault(constants::k_focal_a_star_parameters, {});
        setDefault(constants::k_conflict_based_search_parameters,
                   {{constants::k_constraint_tree_node_cost_type, ConstraintTreeNodeCostType::e_makespan}});
    }
}  // namespace grstapse