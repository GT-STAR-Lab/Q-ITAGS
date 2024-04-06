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
#include "grstapse/common/search/best_first_search_base.hpp"
#include "grstapse/common/search/best_first_search_functors.hpp"
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search_node_base.hpp"

namespace grstapse
{
    /*!
     * \brief Conducts a generic Greedy Best First Search (GBFS)
     *
     * \tparam SearchNode A derivative of GreedyBestFirstSearchNodeBase
     * \tparam SearchStatistics A derivative of SearchStatisticsBase
     */
    template <GreedyBestFirstSearchNodeDeriv SearchNode,
              SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class GreedyBestFirstSearch : public BestFirstSearchBase<SearchNode, SearchStatistics>
    {
        using Base_ = BestFirstSearchBase<SearchNode, SearchStatistics>;

       public:
        /*!
         * \brief Constructor
         *
         * \param parameters
         * \param functors
         */
        explicit GreedyBestFirstSearch(const std::shared_ptr<const ParametersBase>& parameters,
                                       const BestFirstSearchFunctors<SearchNode>& functors)
            : Base_{.parameters = parameters, .functors = functors}
        {}

        /*!
         * Constructor
         *
         * \param parameters
         * \param functors
         */
        explicit GreedyBestFirstSearch(const std::shared_ptr<const ParametersBase>& parameters,
                                       BestFirstSearchFunctors<SearchNode>&& functors)
            : Base_{parameters, functors}
        {}

       protected:
        //! Computes the heuristic value for a node
        virtual void evaluateNode(const std::shared_ptr<SearchNode>& child) final override
        {
            TimerRunner timer_runner(Base_::m_parameters->template get<std::string>(constants::k_timer_name) +
                                     "_heuristic");
            child->setH(Base_::m_heuristic->operator()(child));
        }
    };
}  // namespace grstapse