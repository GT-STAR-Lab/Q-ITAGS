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

// External
#include <nlohmann/json.hpp>

#include <grstapse/geometric_planning/query_results/graph_motion_planner_query_result.hpp>
#include <grstapse/geometric_planning/query_results/motion_planner_query_result_base.hpp>

// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    enum class ConfigurationType : uint8_t
    {
        e_unknown = 0,
        e_ompl,
        e_graph
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(ConfigurationType,
                                 {{ConfigurationType::e_unknown, "unknown"},
                                  {ConfigurationType::e_ompl, "ompl"},
                                  {ConfigurationType::e_graph, "graph"}})

    //! An enumeration of OMPL state spaces
    enum class OmplStateSpaceType : uint8_t
    {
        e_unknown = 0,
        e_se2,
        e_se3,
        e_so3
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplStateSpaceType,
                                 {{OmplStateSpaceType::e_unknown, "unknown"},
                                  {OmplStateSpaceType::e_se2, "se2"},
                                  {OmplStateSpaceType::e_se3, "se3"},
                                  {OmplStateSpaceType::e_so3, "so3"}})

    /*!
     * \brief An enumeration of the possible OMPL goal types
     *
     * \note We currently only support e_state (algorithm limitation, not a coding one)
     */
    enum class OmplGoalType : uint8_t
    {
        e_unknown = 0,
        e_state,
        e_set_of_states,
        e_space
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplGoalType,
                                 {{OmplGoalType::e_unknown, "unknown"},
                                  {OmplGoalType::e_state, "state"},
                                  {OmplGoalType::e_set_of_states, "set_of_states"},
                                  {OmplGoalType::e_space, "space"}})

    enum class OmplEnvironmentType : uint8_t
    {
        e_unknown = 0,
        e_pgm
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplEnvironmentType,
                                 {{OmplEnvironmentType::e_unknown, "unknown"}, {OmplEnvironmentType::e_pgm, "pgm"}})

    //! An enumeration of motion planner algorithms
    enum class OmplMotionPlannerType : uint8_t
    {
        e_unknown = 0,
        e_prm,
        e_prm_star,
        e_lazy_prm,
        e_lazy_prm_star,
        e_rrt,
        e_rrt_star,
        e_parallel_rrt,
        e_rrt_connect,
        e_lazy_rrt
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(OmplMotionPlannerType,
                                 {{OmplMotionPlannerType::e_unknown, "unknown"},
                                  {OmplMotionPlannerType::e_prm, "prm"},
                                  {OmplMotionPlannerType::e_prm_star, "prm_star"},
                                  {OmplMotionPlannerType::e_lazy_prm, "lazy_prm"},
                                  {OmplMotionPlannerType::e_lazy_prm_star, "lazy_prm_star"},
                                  {OmplMotionPlannerType::e_rrt, "rrt"},
                                  {OmplMotionPlannerType::e_rrt_star, "rrt_star"},
                                  {OmplMotionPlannerType::e_parallel_rrt, "parallel_rrt"},
                                  {OmplMotionPlannerType::e_rrt_connect, "rrt_connect"},
                                  {OmplMotionPlannerType::e_lazy_rrt, "lazy_rrt"}})

    //! The type of graph
    enum class GraphType : uint8_t
    {
        e_unknown = 0,
        e_euclidean,
        e_grid
    };

    NLOHMANN_JSON_SERIALIZE_ENUM(GraphType,
                                 {{GraphType::e_unknown, "unknown"},
                                  {GraphType::e_euclidean, "euclidean"},
                                  {GraphType::e_grid, "grid"}})

    //! The type of euclidean graph
    enum class EuclideanGraphType : uint8_t
    {
        e_unknown = 0,
        e_singular,
        e_sampled
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(EuclideanGraphType,
                                 {{EuclideanGraphType::e_unknown, "unknown"},
                                  {EuclideanGraphType::e_singular, "singular"},
                                  {EuclideanGraphType::e_sampled, "sampled"}})

    //! An enumeration of possible statuses resulting from a motion planning query
    enum class MotionPlannerQueryStatus : uint8_t
    {
        e_unknown = 0,
        e_success,
        e_failure,
        e_timeout
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(MotionPlannerQueryStatus,
                                 {{MotionPlannerQueryStatus::e_unknown, "unknown"},
                                  {MotionPlannerQueryStatus::e_success, "success"},
                                  {MotionPlannerQueryStatus::e_failure, "failure"},
                                  {MotionPlannerQueryStatus::e_timeout, "timeout"}});
}  // namespace grstapse