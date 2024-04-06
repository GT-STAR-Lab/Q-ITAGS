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
#include "grstapse/geometric_planning/mapf/cbs/conflict_based_search.hpp"

// Local
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/conflict_base.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_root.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/edge_conflict.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_conflict.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_constraint.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/space_time_a_star_with_constraints.hpp"
#include "grstapse/parameters/parameters_factory.hpp"
#include "grstapse/problem_inputs/multi_agent_path_finding_problem_inputs.hpp"

namespace grstapse
{
    ConflictBaseSearch::ConflictBaseSearch(
        const std::shared_ptr<const MultiAgentPathFindingProblemInputs>& problem_inputs,
        const std::shared_ptr<const ParametersBase>& parameters)
        : Base_(parameters)
        , m_problem_inputs(problem_inputs)
    {}

    std::shared_ptr<ConstraintTreeNodeBase> ConflictBaseSearch::createRootNode()
    {
        return std::make_shared<ConstraintTreeNodeRoot>(
            m_problem_inputs->numberOfRobots(),
            m_parameters->get<ConstraintTreeNodeCostType>(constants::k_constraint_tree_node_cost_type));
    }

    SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics> ConflictBaseSearch::searchFromNode(
        const std::shared_ptr<ConstraintTreeNodeBase>& root)
    {
        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        const ConstraintTreeNodeCostType cost_type =
            m_parameters->get<ConstraintTreeNodeCostType>(constants::k_constraint_tree_node_cost_type);
        const bool has_timeout       = Base_::m_parameters->template get<bool>(constants::k_has_timeout);
        const std::string timer_name = Base_::m_parameters->template get<std::string>(constants::k_timer_name);
        const float timeout          = Base_::m_parameters->template get<float>(constants::k_timeout);

        if(!computeLowLevelSolution(root))
        {
            return SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>(nullptr, Base_::m_statistics);
        }
        Base_::m_statistics->incrementNumberOfHighLevelNodesGenerated();
        m_open.push(root->id(), root);

        while(!m_open.empty())
        {
            // Timed out
            if(has_timeout && TimeKeeper::instance().time(timer_name) > timeout)
            {
                Logger::warn("Search exceeded the timeout");
                break;
            }

            std::shared_ptr<ConstraintTreeNodeBase> base = m_open.pop();

            std::unique_ptr<const ConflictBase> conflict = base->getFirstConflict();
            if(conflict == nullptr)
            {
                return SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>(base, Base_::m_statistics);
            }
            base->setStatus(SearchNodeStatus::e_closed);

            robin_hood::unordered_map<unsigned int, std::shared_ptr<ConstraintBase>> constraints =
                conflict->createConstraints();
            for(const auto& [robot, constraint]: constraints)
            {
                auto child = std::make_shared<ConstraintTreeNode>(num_robots, cost_type, base);
                child->setConstraint(robot, constraint);
                Base_::m_statistics->incrementNumberOfHighLevelNodesGenerated();
                if(computeLowLevelSolution(child, robot))
                {
                    m_open.push(child->id(), child);
                    child->setStatus(SearchNodeStatus::e_open);
                }
                Base_::m_statistics->incrementNumberOfHighLevelNodesEvaluated();
            }
        }

        return SearchResults<ConstraintTreeNodeBase, ConflictBasedSearchStatistics>(nullptr, Base_::m_statistics);
    }

    bool ConflictBaseSearch::computeLowLevelSolution(const std::shared_ptr<ConstraintTreeNodeBase>& node)
    {
        for(unsigned int i = 0, num_robots = m_problem_inputs->numberOfRobots(); i < num_robots; ++i)
        {
            if(!computeLowLevelSolution(node, i))
            {
                return false;
            }
        }
        return true;
    }

    bool ConflictBaseSearch::computeLowLevelSolution(const std::shared_ptr<ConstraintTreeNodeBase>& node,
                                                     unsigned int robot)
    {
        auto low_level_parameters = ParametersFactory::instance().create(
            ParametersFactory::Type::e_search,
            {{constants::k_config_type, constants::k_best_first_search_parameters},
             {constants::k_has_timeout, m_parameters->get<bool>(constants::k_has_timeout)},
             {constants::k_timeout,
              m_parameters->get<float>(constants::k_timeout) -
                  TimeKeeper::instance().time(m_parameters->get<std::string>(constants::k_timer_name))},
             {constants::k_timer_name, m_parameters->get<std::string>(constants::k_low_level_timer_name)}});
        SpaceTimeAStarWithConstraints low_level(low_level_parameters,
                                                m_problem_inputs->map(),
                                                m_problem_inputs->initialStates()[robot],
                                                m_problem_inputs->goalStates()[robot],
                                                node->constraints(robot));
        SearchResults<TemporalGridCellNode, SearchStatisticsCommon> result = low_level.search();
        std::shared_ptr<SearchStatisticsCommon> low_level_statistics       = result.statistics();
        Base_::m_statistics->incrementNumberOfLowLevelNodesGenerated(low_level_statistics->numberOfNodesGenerated());
        Base_::m_statistics->incrementNumberOfLowLevelNodesEvaluated(low_level_statistics->numberOfNodesEvaluated());
        Base_::m_statistics->incrementNumberOfLowLevelNodesExpanded(low_level_statistics->numberOfNodesExpanded());
        if(!result.foundGoal())
        {
            return false;
        }
        node->setLowLevelSolution(robot, result.goal());
        return true;
    }
}  // namespace grstapse