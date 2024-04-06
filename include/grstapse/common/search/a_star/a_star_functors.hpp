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
#include <concepts>
#include <memory>

// Local
#include "grstapse/common/search/a_star/a_star_search_node_base.hpp"
#include "grstapse/common/search/best_first_search_functors.hpp"
#include "grstapse/common/search/path_cost_base.hpp"

namespace grstapse
{
    namespace detail
    {
        //! \brief Hack for designated initializers
        template <AStarSearchNodeDeriv SearchNode>
        struct AStarFunctorParametersImpl
        {
            using PathCost           = PathCostBase<SearchNode>;
            using Heuristic          = HeuristicBase<SearchNode>;
            using SuccessorGenerator = SuccessorGeneratorBase<SearchNode>;
            using GoalCheck          = GoalCheckBase<SearchNode>;
            using Memoization        = MemoizationBase<SearchNode>;
            using PruningMethod      = PruningMethodBase<SearchNode>;

            std::shared_ptr<const PathCost> path_cost;
            std::shared_ptr<const Heuristic> heuristic;
            std::shared_ptr<const SuccessorGenerator> successor_generator;
            std::shared_ptr<const GoalCheck> goal_check;
            std::shared_ptr<const Memoization> memoization    = std::make_shared<const NullMemoization<SearchNode>>();
            std::shared_ptr<PruningMethod> prepruning_method  = std::make_shared<NullPruningMethod<SearchNode>>();
            std::shared_ptr<PruningMethod> postpruning_method = std::make_shared<NullPruningMethod<SearchNode>>();
        };
    }  // namespace detail

    /*!
     * \brief A container for functors used by A*
     *
     * \tparam SearchNode A derivative of AStarSearchNodeBase
     */
    template <AStarSearchNodeDeriv SearchNode>
    class AStarFunctors : public BestFirstSearchFunctors<SearchNode>
    {
        using Base = BestFirstSearchFunctors<SearchNode>;

       public:
        using PathCost = PathCostBase<SearchNode>;

        /*!
         * \brief Constructor
         *
         * \param parameters A hack to get designated parameters
         */
        AStarFunctors(const detail::AStarFunctorParametersImpl<SearchNode>& parameters)
            : Base{.heuristic           = parameters.heuristic,
                   .successor_generator = parameters.successor_generator,
                   .goal_check          = parameters.goal_check,
                   .memoization         = parameters.memoization,
                   .prepruning_method   = parameters.prepruning_method,
                   .postpruning_method  = parameters.postpruning_method}
            , path_cost(parameters.path_cost)
        {}

        /*!
         * \brief Constructor
         *
         * \param parameters A hack to get designated parameters
         */
        AStarFunctors(detail::AStarFunctorParametersImpl<SearchNode>&& parameters)
            : Base{.heuristic           = std::move(parameters.heuristic),
                   .successor_generator = std::move(parameters.successor_generator),
                   .goal_check          = std::move(parameters.goal_check),
                   .memoization         = std::move(parameters.memoization),
                   .prepruning_method   = std::move(parameters.prepruning_method),
                   .postpruning_method  = std::move(parameters.postpruning_method)}
            , path_cost(parameters.path_cost)
        {}

        std::shared_ptr<const PathCost> path_cost;
    };

}  // namespace grstapse