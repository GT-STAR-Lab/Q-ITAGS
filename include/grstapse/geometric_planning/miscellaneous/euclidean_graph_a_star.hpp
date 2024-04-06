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

// Local
#include "grstapse/common/search/a_star/a_star.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_a_star_search_node.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment.hpp"

namespace grstapse
{
    //! An A* search on a point graph
    class EuclideanGraphAStar : public AStar<UndirectedGraphAStarSearchNode<EuclideanGraphConfiguration>>
    {
        using SearchNode_ = UndirectedGraphAStarSearchNode<EuclideanGraphConfiguration>;
        using Base_       = AStar<SearchNode_>;

       public:
        //! Constructor
        explicit EuclideanGraphAStar(const std::shared_ptr<const ParametersBase>& parameters,
                                     const std::shared_ptr<const EuclideanGraphConfiguration>& root,
                                     const std::shared_ptr<EuclideanGraphEnvironment>& graph,
                                     const AStarFunctors<SearchNode_>& functors);

        //! \copydoc BestFirstSearchBase
        std::shared_ptr<SearchNode_> createRootNode() final override;

       private:
        std::shared_ptr<SearchNode_> m_root;
    };
}  // namespace grstapse