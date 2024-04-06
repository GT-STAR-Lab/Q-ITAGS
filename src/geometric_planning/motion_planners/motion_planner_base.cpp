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
#include "grstapse/geometric_planning/motion_planners/motion_planner_base.hpp"

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/common/utilities/json_tree_factory.hpp"
#include "grstapse/common/utilities/timer_runner.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/complete_euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/masked_complete_sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"
#include "grstapse/geometric_planning/query_results/motion_planner_query_result_base.hpp"
#include "grstapse/parameters/parameters_base.hpp"
#include "grstapse/parameters/parameters_factory.hpp"

namespace grstapse
{
    unsigned int MotionPlannerBase::s_num_failures = 0;

    MotionPlannerBase::MotionPlannerBase(const std::shared_ptr<const ParametersBase>& parameters,
                                         const std::shared_ptr<EnvironmentBase>& environment)
        : m_parameters(parameters)
        , m_environment(environment)
    {}

    void MotionPlannerBase::init()
    {
        static bool first = true;
        if(first)
        {
            first                                 = false;
            ParametersFactory& parameters_factory = ParametersFactory::instance();
            OmplEnvironment::init();

            // region CompleteEuclideanGraphMotionPlanner
            JsonTreeFactory<MotionPlannerBase>::instance().set(
                constants::k_complete_euclidean_graph_motion_planner,
                [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<MotionPlannerBase>
                {
                    json_ext::validateJson(j,
                                           {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                                            {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                    auto parameters  = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                j.at(constants::k_algorithm_parameters));
                    auto environment = j.at(constants::k_environment_parameters)
                                           .get<std::shared_ptr<grstapse::EuclideanGraphEnvironment>>();
                    return std::make_shared<CompleteEuclideanGraphMotionPlanner>(parameters, environment);
                });
            // endregion
            // region CompleteSampledEuclideanGraphMotionPlanner
            JsonTreeFactory<MotionPlannerBase>::instance().set(
                constants::k_complete_sampled_euclidean_graph_motion_planner,
                [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<MotionPlannerBase>
                {
                    json_ext::validateJson(j,
                                           {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                                            {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                    auto parameters = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                j.at(constants::k_algorithm_parameters));
                    const nlohmann::json& environment_j = j.at(constants::k_environment_parameters);
                    auto environment = environment_j.get<std::shared_ptr<grstapse::SampledEuclideanGraphEnvironment>>();

                    return std::make_shared<CompleteSampledEuclideanGraphMotionPlanner>(parameters, environment);
                });
            // endregion
            // region MaskedCompleteSampledEuclideanGraphMotionPlanner
            JsonTreeFactory<MotionPlannerBase>::instance().set(
                constants::k_masked_complete_sampled_euclidean_graph_motion_planner,
                [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<MotionPlannerBase>
                {
                    json_ext::validateJson(j,
                                           {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                                            {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                    auto parameters = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                j.at(constants::k_algorithm_parameters));
                    const nlohmann::json& environment_j = j.at(constants::k_environment_parameters);
                    auto environment = environment_j.get<std::shared_ptr<grstapse::SampledEuclideanGraphEnvironment>>();

                    return std::make_shared<MaskedCompleteSampledEuclideanGraphMotionPlanner>(parameters, environment);
                });
            // endregion
            // region SampledEuclideanGraphMotionPlanner
            JsonTreeFactory<MotionPlannerBase>::instance().set(
                constants::k_sampled_euclidean_graph_motion_planner,
                [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<MotionPlannerBase>
                {
                    json_ext::validateJson(j,
                                           {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                                            {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                    auto parameters  = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                j.at(constants::k_algorithm_parameters));
                    auto environment = j.at(constants::k_environment_parameters)
                                           .get<std::shared_ptr<grstapse::SampledEuclideanGraphEnvironment>>();
                    return std::make_shared<SampledEuclideanGraphMotionPlanner>(parameters, environment);
                });
            // endregion
            // region EuclideanGraphMotionPlanner
            JsonTreeFactory<MotionPlannerBase>::instance().set(
                constants::k_euclidean_graph_motion_planner,
                [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<MotionPlannerBase>
                {
                    json_ext::validateJson(j,
                                           {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                                            {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                    auto parameters  = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                j.at(constants::k_algorithm_parameters));
                    auto environment = j.at(constants::k_environment_parameters)
                                           .get<std::shared_ptr<grstapse::EuclideanGraphEnvironment>>();
                    return std::make_shared<EuclideanGraphMotionPlanner>(parameters, environment);
                });
            // endregion
            // region OmplMotionPlanner
            JsonTreeFactory<MotionPlannerBase>::instance().set(
                constants::k_ompl_motion_planner,
                [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<MotionPlannerBase>
                {
                    grstapse::json_ext::validateJson(
                        j,
                        {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                         {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                    auto parameters = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                j.at(constants::k_algorithm_parameters));

                    auto environment = JsonTreeFactory<OmplEnvironment>::instance().create(
                        j.at(grstapse::constants::k_environment_parameters));
                    return std::make_shared<OmplMotionPlanner>(
                        parameters->get<grstapse::OmplMotionPlannerType>(constants::k_ompl_mp_algorithm),
                        parameters,
                        environment);
                });
            // endregion
        }
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> MotionPlannerBase::query(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        std::lock_guard lock(m_mutex);
        TimerRunner timer_runner(constants::k_motion_planning_time);
        if(std::shared_ptr<const MotionPlannerQueryResultBase> result =
               getMemoized(species, initial_configuration, goal_configuration);
           result != nullptr)
        {
            return result;
        }

        // Compute and memoize
        std::shared_ptr<const MotionPlannerQueryResultBase> result =
            computeMotionPlan(species, initial_configuration, goal_configuration);
        auto iter = m_memoization.emplace(std::weak_ptr<const Species>(species),
                                          std::make_tuple(initial_configuration, goal_configuration, result));
        return result;
    }

    float MotionPlannerBase::durationQuery(const std::shared_ptr<const Species>& species,
                                           const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                           const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        if(std::shared_ptr<const MotionPlannerQueryResultBase> result =
               query(species, initial_configuration, goal_configuration);
           result != nullptr)
        {
            return result->duration(species->speed());
        }
        return -1.0f;
    }

    bool MotionPlannerBase::isMemoized(const std::shared_ptr<const Species>& species,
                                       const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                       const std::shared_ptr<const ConfigurationBase>& goal_configuration) const
    {
        std::lock_guard lock(m_mutex);
        TimerRunner timer_runner(constants::k_motion_planning_time);
        return getMemoized(species, initial_configuration, goal_configuration) != nullptr;
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> MotionPlannerBase::getMemoized(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration) const
    {
        if(!m_memoization.contains(species))
        {
            return nullptr;
        }

        auto range = m_memoization.equal_range(species);
        for(auto iter = range.first; iter != range.second; ++iter)
        {
            const MemoizationValue& mv = iter->second;
            if(*std::get<0>(mv) != *initial_configuration)
            {
                continue;
            }
            if(*std::get<1>(mv) != *goal_configuration)
            {
                continue;
            }
            return std::get<2>(mv);
        }

        return nullptr;
    }

    void MotionPlannerBase::clearCache()
    {
        m_memoization.clear();
    }

    unsigned int MotionPlannerBase::numFailures()
    {
        return s_num_failures;
    }
}  // namespace grstapse