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
#include "grstapse/problem_inputs/grstaps_problem_inputs.hpp"

// Global
#include <fstream>
// External
#include <fmt/format.h>
// Local
#include "grstapse/common/milp/milp_solver_base.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/common/utilities/json_tree_factory.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/configurations/se2_ompl_configuration.hpp"
#include "grstapse/geometric_planning/configurations/se3_ompl_configuration.hpp"
#include "grstapse/geometric_planning/environments/graph_environment.hpp"
#include "grstapse/geometric_planning/environments/ompl_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"
#include "grstapse/parameters/parameters_factory.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"
#include "grstapse/task_planning/sas/sas_action.hpp"

namespace grstapse
{
    GrstapsProblemInputs::GrstapsProblemInputs(const ThisIsProtectedTag&)
        : m_task_configuration_type(ConfigurationType::e_unknown)
        , m_ompl_state_space_type(OmplStateSpaceType::e_unknown)
        , m_graph_type(GraphType::e_unknown)
    {}

    GrstapsProblemInputs::~GrstapsProblemInputs()
    {
        // Clear the created gurobi environments
        MilpSolverBase::clearEnvironments();
        // Clear the mp cache to remove shared_ptr cycle species->mp->species
        for(std::shared_ptr<MotionPlannerBase>& motion_planner: m_motion_planners)
        {
            motion_planner->clearCache();
        }
    }

    void GrstapsProblemInputs::checkConfiguration(const std::shared_ptr<const ConfigurationBase>& configuration) const
    {
        if(configuration->configurationType() != m_task_configuration_type)
        {
            throw createLogicError("Configuration type does not match the central one");
        }

        switch(m_task_configuration_type)
        {
            case ConfigurationType::e_ompl:
            {
                switch(m_ompl_state_space_type)
                {
                    case OmplStateSpaceType::e_se2:
                    {
                        const auto& se2_configuration =
                            std::dynamic_pointer_cast<const Se2OmplConfiguration>(configuration);
                        if(!se2_configuration)
                        {
                            throw createLogicError("Configuration state space type does not match the central one");
                        }
                        break;
                    }
                    case OmplStateSpaceType::e_se3:
                    {
                        const auto& se3_configuration =
                            std::dynamic_pointer_cast<const Se3OmplConfiguration>(configuration);
                        if(!se3_configuration)
                        {
                            throw createLogicError("Configuration state space type does not match the central one");
                        }
                        break;
                    }
                    default:
                    {
                        throw createLogicError("Unknown ompl state space type");
                    }
                }
                break;
            }
            case ConfigurationType::e_graph:
            {
                switch(m_graph_type)
                {
                    case GraphType::e_euclidean:
                    {
                        auto pgc = std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(configuration);
                        if(!pgc)
                        {
                            throw createLogicError("Configuration graph type does not match the central one");
                        }
                        break;
                    }
                    case GraphType::e_grid:
                    {
                        throw createLogicError("Grid Configurations Not Implemented");
                    }
                    default:
                    {
                        throw createLogicError("Unknown graph type");
                    }
                }
                break;
            }
            default:
            {
                throw createLogicError("Unknown task configuration type");
            }
        }
    }

    void GrstapsProblemInputs::loadMotionPlanners(const nlohmann::json& j)
    {
        if(!j.is_array())
        {
            throw createLogicError("'motion_planners' should be an array of objects");
        }

        MotionPlannerBase::init();
        m_motion_planners.reserve(j.size());
        for(const nlohmann::json& individual_mp: j)
        {
            m_motion_planners.emplace_back(JsonTreeFactory<MotionPlannerBase>::instance().create(individual_mp));

            // First MP
            if(m_task_configuration_type == ConfigurationType::e_unknown)
            {
                m_task_configuration_type = m_motion_planners.back()->environment()->configurationType();
            }
            // Any that do not agree with previous motion planners on the configuration space
            else if(m_task_configuration_type != m_motion_planners.back()->environment()->configurationType())
            {
                throw createLogicError("Cannot load environments of different configuration types");
            }

            if(m_task_configuration_type == ConfigurationType::e_ompl)
            {
                const auto& ompl_environment =
                    std::dynamic_pointer_cast<OmplEnvironment>(m_motion_planners.back()->environment());
                if(m_ompl_state_space_type == OmplStateSpaceType::e_unknown)
                {
                    m_ompl_state_space_type = ompl_environment->stateSpaceType();
                }
                else if(m_ompl_state_space_type != ompl_environment->stateSpaceType())
                {
                    throw createLogicError("Cannot load OMPL environments with different state space types");
                }
            }
            else if(m_task_configuration_type == ConfigurationType::e_graph)
            {
                const auto& graph_environment =
                    std::dynamic_pointer_cast<GraphEnvironment>(m_motion_planners.back()->environment());
                if(m_graph_type == GraphType::e_unknown)
                {
                    m_graph_type = graph_environment->graphType();
                }
                else if(m_graph_type != graph_environment->graphType())
                {
                    throw createLogicError("Cannot load graph environments with different graph types");
                }
            }
        }
    }

    void GrstapsProblemInputs::createTasks(const std::vector<std::shared_ptr<SasAction>>& grounded_sas_actions,
                                           const nlohmann::json& j)
    {
        m_tasks.reserve(grounded_sas_actions.size());
        for(const std::shared_ptr<SasAction>& action: grounded_sas_actions)
        {
            if(!j.contains(action->name()))
            {
                throw createLogicError(
                    fmt::format("No associated trait or geometric data for task '{0:s}'", action->name()));
            }

            const nlohmann::json& task_associations_j = j.at(action->name());

            const Eigen::VectorXf desired_traits =
                task_associations_j.at(constants::k_desired_traits).get<Eigen::VectorXf>();
            const Eigen::VectorXf linear_coefficients =
                task_associations_j.at(constants::k_linear_quality_coefficients).get<Eigen::VectorXf>();

            const nlohmann::json& initial_configuration_j = task_associations_j.at(constants::k_initial_configuration);
            const nlohmann::json& terminal_configuration_j =
                task_associations_j.at(constants::k_terminal_configuration);

            const std::shared_ptr<const ConfigurationBase> initial_configuration =
                initial_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            checkConfiguration(initial_configuration);

            const std::shared_ptr<const ConfigurationBase> terminal_configuration =
                terminal_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            checkConfiguration(terminal_configuration);

            m_tasks.push_back(std::make_shared<const Task>(action,
                                                           desired_traits,
                                                           initial_configuration,
                                                           terminal_configuration,
                                                           linear_coefficients));
        }
    }
    std::pair<std::map<std::string, std::shared_ptr<const Species>>, unsigned int> GrstapsProblemInputs::loadSpecies(
        const nlohmann::json& j)
    {
        if(m_motion_planners.empty())
        {
            Logger::warn("Loading species without loading motion planners first");
        }

        std::map<std::string, std::shared_ptr<const Species>> rv;
        unsigned int num_traits;
        bool num_traits_set = false;

        m_species.reserve(j.size());
        for(const nlohmann::json& species_j: j)
        {
            m_species.push_back(Species::loadJson(species_j, m_motion_planners));
        }
        // Build map to use for loading the robots down below
        for(const std::shared_ptr<const Species>& s: m_species)
        {
            rv[s->name()] = s;
            if(!num_traits_set)
            {
                num_traits     = s->traits().size();
                num_traits_set = true;
            }
        }

        return {rv, num_traits};
    }

    void GrstapsProblemInputs::loadRobots(
        const std::map<std::string, std::shared_ptr<const Species>>& name_to_species_mapping,
        const unsigned int num_traits,
        const nlohmann::json& j)
    {
        unsigned int num_robots = j.size();
        unsigned int robot_nr   = 0;

        m_robots.reserve(num_robots);
        m_team_traits_matrix.resize(num_robots, num_traits);
        for(const nlohmann::json robot_j: j)
        {
            const std::string name = robot_j.at(constants::k_name).get<std::string>();
            std::shared_ptr<const ConfigurationBase> initial_configuration =
                robot_j.at(constants::k_initial_configuration).get<std::shared_ptr<ConfigurationBase>>();
            const std::string species_name = robot_j.at(constants::k_species).get<std::string>();

            m_robots.push_back(
                std::make_shared<const Robot>(name, initial_configuration, name_to_species_mapping.at(species_name)));
            m_team_traits_matrix.row(robot_nr++) = m_robots.back()->species()->traits();
        }
    }
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<grstapse::GrstapsProblemInputs>
    nlohmann::adl_serializer<std::shared_ptr<grstapse::GrstapsProblemInputs>>::from_json(const nlohmann::json& j)
    {
        auto problem_inputs =
            std::make_shared<grstapse::GrstapsProblemInputs>(grstapse::GrstapsProblemInputs::s_this_is_protected_tag);

        // TODO(Andrew): Load PDDL files
        const std::string pddl_domain_filepath =
            j.at(grstapse::constants::k_pddl).at(grstapse::constants::k_domain_filepath).get<std::string>();
        const std::string pddl_problem_filepath =
            j.at(grstapse::constants::k_pddl).at(grstapse::constants::k_problem_filepath).get<std::string>();
        // Parse PDDL
        std::vector<std::shared_ptr<grstapse::SasAction>> grounded_sas_actions;

        // Load Environments & Motion Planners
        problem_inputs->loadMotionPlanners(j.at(grstapse::constants::k_motion_planners));

        // Load Task Associations
        problem_inputs->createTasks(grounded_sas_actions, j.at(grstapse::constants::k_task_associations));

        // Load Species
        auto [name_to_species_mapping, num_traits] = problem_inputs->loadSpecies(j.at(grstapse::constants::k_species));

        // Load Robots
        // Note: cannot use the normal from_json function because the vector of species is needed
        problem_inputs->loadRobots(name_to_species_mapping, num_traits, j.at(grstapse::constants::k_robots));

        // Load Module Parameters
        {
            problem_inputs->m_fcpop_parameters =
                grstapse::ParametersFactory::instance().create(grstapse::ParametersFactory::Type::e_search,
                                                               j.at(grstapse::constants::k_fcpop_parameters));
            problem_inputs->m_itags_parameters =
                grstapse::ParametersFactory::instance().create(grstapse::ParametersFactory::Type::e_search,
                                                               j.at(grstapse::constants::k_itags_parameters));
            if(j.find(grstapse::constants::k_robot_traits_matrix_reduction) != j.end())
            {
                problem_inputs->m_robot_traits_matrix_reduction =
                    j.at(grstapse::constants::k_robot_traits_matrix_reduction)
                        .get<std::shared_ptr<grstapse::RobotTraitsMatrixReduction>>();
            }
            else
            {
                problem_inputs->m_robot_traits_matrix_reduction =
                    std::make_shared<const grstapse::RobotTraitsMatrixReduction>();
            }
            problem_inputs->m_scheduler_parameters =
                grstapse::ParametersFactory::instance().create(grstapse::ParametersFactory::Type::e_scheduler,
                                                               j.at(grstapse::constants::k_scheduler_parameters));
            // MP parameters are loaded up above
        }

        return problem_inputs;
    }
}  // namespace nlohmann