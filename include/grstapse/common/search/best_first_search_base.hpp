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

// region Includes
// Global
#include <cassert>
#include <memory>
#include <set>
#include <vector>
// Local
#include "grstapse/common/mutable_priority_queue/mutable_priority_queue.hpp"
#include "grstapse/common/search/best_first_search_functors.hpp"
#include "grstapse/common/search/best_first_search_node_base.hpp"
#include "grstapse/common/search/search_algorithm_base.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/parameters/parameters_base.hpp"
// endregion

namespace grstapse
{
    /*!
     * \brief An abstract base class for all best first searches (A*, Greedy Best First Search, etc)
     *
     * \tparam SearchNode A derivative of BestFirstSearchNodeBase
     * \tparam SearchStatistics A derivative of SearchStatisticsBase
     *
     * \todo Add continue search function for anytime
     */
    template <BestFirstSearchNodeDeriv SearchNode, SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class BestFirstSearchBase : public SearchAlgorithmBase<SearchNode, SearchStatistics>
    {
        using Base_               = SearchAlgorithmBase<SearchNode, SearchStatistics>;
        using Heuristic_          = HeuristicBase<SearchNode>;
        using SuccessorGenerator_ = SuccessorGeneratorBase<SearchNode>;
        using GoalCheck_          = GoalCheckBase<SearchNode>;
        using Memoization_        = MemoizationBase<SearchNode>;
        using PruningMethod_      = PruningMethodBase<SearchNode>;

       public:
        /*!
         * \brief Constructor
         *
         * \param parameters The parameters for a best first search
         * \param functors A container for the various functors used by best first search (heuristic, goal check, etc)
         */
        explicit BestFirstSearchBase(const std::shared_ptr<const ParametersBase>& parameters,
                                     const BestFirstSearchFunctors<SearchNode>& functors)
            : Base_(parameters)
            , m_heuristic(functors.heuristic)
            , m_successor_generator(functors.successor_generator)
            , m_goal_check(functors.goal_check)
            , m_memoization(functors.memoization)
            , m_prepruning_method(functors.prepruning_method)
            , m_postpruning_method(functors.postpruning_method)
        {}

        /*!
         * \brief Constructor
         *
         * \param parameters The parameters for a best first search
         * \param functors A container for the various functors used by best first search (heuristic, goal check, etc)
         */
        explicit BestFirstSearchBase(const std::shared_ptr<const ParametersBase>& parameters,
                                     BestFirstSearchFunctors<SearchNode>&& functors)
            : Base_(parameters)
            , m_heuristic{std::move(functors.heuristic)}
            , m_successor_generator{std::move(functors.successor_generator)}
            , m_goal_check{std::move(functors.goal_check)}
            , m_memoization{std::move(functors.memoization)}
            , m_prepruning_method{std::move(functors.prepruning_method)}
            , m_postpruning_method{std::move(functors.postpruning_method)}
        {}

        /*!
         * \brief Runs the search
         *
         * \returns The results of the search (solution and statistics)
         */
        SearchResults<SearchNode, SearchStatistics> searchFromNode(const std::shared_ptr<SearchNode>& root) override
        {
            assert(root);
            evaluateNode(root);
            Base_::m_statistics->incrementNodesGenerated();
            m_open.push(m_memoization->operator()(root), root);

            const bool has_prepruning  = m_prepruning_method != nullptr;
            const bool has_postpruning = m_postpruning_method != nullptr;

            const bool has_timeout       = Base_::m_parameters->template get<bool>(constants::k_has_timeout);
            const std::string timer_name = Base_::m_parameters->template get<std::string>(constants::k_timer_name);
            const float timeout          = Base_::m_parameters->template get<float>(constants::k_timeout);

            const bool save_closed_nodes = Base_::m_parameters->template get<bool>(constants::k_save_closed_nodes);
            const bool save_pruned_nodes = Base_::m_parameters->template get<bool>(constants::k_save_pruned_nodes);

            // Continue through open set until it is empty or timeout
            // Only check timeout if parameter is set
            while(!m_open.empty())
            {
                // Timed out
                if(has_timeout && TimeKeeper::instance().time(timer_name) > timeout)
                {
                    Logger::warn("Search exceeded the timeout");
                    break;
                }

                std::shared_ptr<SearchNode> base = m_open.pop();

                // Close node before the goal check for future anytime/repair
                if(save_closed_nodes)
                {
                    m_closed.push_back(base);
                }
                m_closed_ids.insert(m_memoization->operator()(base));
                base->setStatus(SearchNodeStatus::e_closed);

                // Check if goal node
                if(m_goal_check->operator()(base))
                {
                    return SearchResults<SearchNode, SearchStatistics>(base, Base_::m_statistics);
                }

                Base_::m_statistics->incrementNodesExpanded();
                bool deadend = true;
                for(std::shared_ptr<SearchNode> child: m_successor_generator->operator()(base))
                {
                    deadend = false;
                    Base_::m_statistics->incrementNodesGenerated();
                    // Timed out
                    if(has_timeout && TimeKeeper::instance().time(timer_name) > timeout)
                    {
                        Logger::warn("Search timed out");
                        break;
                    }

                    const unsigned int id = m_memoization->operator()(child);

                    // Ignore if this node has already been closed or pruned
                    if(m_closed_ids.find(id) != m_closed_ids.end() || m_pruned_ids.find(id) != m_pruned_ids.end())
                    {
                        continue;
                    }

                    // Check if the child should be pruned before evaluation
                    if(has_prepruning && m_prepruning_method->operator()(child))
                    {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        Base_::m_statistics->incrementNodesPruned();
                        m_pruned_ids.insert(id);
                        if(save_pruned_nodes)
                        {
                            m_pruned.push_back(child);
                        }
                        continue;
                    }

                    // Evaluate
                    evaluateNode(child);
                    Base_::m_statistics->incrementNodesEvaluated();

                    // Check if child should be pruned after evaluation
                    if(has_postpruning && m_postpruning_method->operator()(child))
                    {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        Base_::m_statistics->incrementNodesPruned();
                        m_pruned_ids.insert(id);
                        if(save_pruned_nodes)
                        {
                            m_pruned.push_back(child);
                        }
                        continue;
                    }

                    // Add child to open set
                    child->setStatus(SearchNodeStatus::e_open);
                    m_open.push(id, child);
                }
                if(deadend)
                {
                    base->setStatus(SearchNodeStatus::e_deadend);
                    Base_::m_statistics->incrementNodesDeadend();
                }
            }
            return SearchResults<SearchNode, SearchStatistics>(nullptr, Base_::m_statistics);
        }

       protected:
        /*!
         * \brief Evaluate the value of a node
         *
         * \param node The node to evaluate
         */
        virtual void evaluateNode(const std::shared_ptr<SearchNode>& node) = 0;

        std::shared_ptr<const Heuristic_> m_heuristic;
        std::shared_ptr<const SuccessorGenerator_> m_successor_generator;
        std::shared_ptr<const GoalCheck_> m_goal_check;
        std::shared_ptr<const Memoization_> m_memoization;
        std::shared_ptr<PruningMethod_> m_prepruning_method;
        std::shared_ptr<PruningMethod_> m_postpruning_method;

        MutablePriorityQueue<unsigned int, float, SearchNode> m_open;  //!< key, priority, payload

        std::vector<std::shared_ptr<SearchNode>> m_closed;
        std::set<unsigned int> m_closed_ids;

        std::vector<std::shared_ptr<SearchNode>> m_pruned;
        std::set<unsigned int> m_pruned_ids;
    };
}  // namespace grstapse