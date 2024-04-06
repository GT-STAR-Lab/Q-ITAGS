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
#include "grstapse/common/search/undirected_graph/undirected_graph.hpp"
#include "grstapse/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "grstapse/geometric_planning/environments/euclidean_graph_environment_base.hpp"

namespace grstapse
{
    // Forward Declaration
    class SampledEuclideanGraphEnvironment;

    //! An environment for an undirected graph were each vertex is point in 2D space
    class EuclideanGraphEnvironment
        : public UndirectedGraph<EuclideanGraphConfiguration>
        , public EuclideanGraphEnvironmentBase
    {
       public:
        //! Default Constructor
        EuclideanGraphEnvironment();

        //! Constructor
        explicit EuclideanGraphEnvironment(bool is_complete);

        //! A shallow copy
        [[nodiscard]] std::shared_ptr<EuclideanGraphEnvironment> shallowCopy() const;

        //! \returns A deep copy with the vertices only
        [[nodiscard]] std::shared_ptr<EuclideanGraphEnvironment> deepCopyVerticesOnly() const;

        //! \return The vertex in the graph that contains \p configuration if it exists, nullptr otherwise
        [[nodiscard]] std::shared_ptr<UndirectedGraphVertex<EuclideanGraphConfiguration>> findPossibleVertex(
            const std::shared_ptr<const EuclideanGraphConfiguration>& configuration) const;

        //! \return The vertex in the graph that contains \p configuration if it exists, throws exception otherwise
        [[nodiscard]] std::shared_ptr<UndirectedGraphVertex<EuclideanGraphConfiguration>> findVertex(
            const std::shared_ptr<const EuclideanGraphConfiguration>& configuration) const;

        //! \returns The edge in the graph that contains \p a and \p b if it exists, nullptr otherwise
        [[nodiscard]] std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> findPossibleEdge(
            const std::shared_ptr<const EuclideanGraphConfiguration>& a,
            const std::shared_ptr<const EuclideanGraphConfiguration>& b) const;

        //! \returns The edge in the graph that contains \p a and \p b if it exists, nullptr otherwise
        [[nodiscard]] std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> findPossibleEdge(
            unsigned int a,
            unsigned int b) const;

        //! \returns The edge in the graph that contains \p a and \p b if it exists, throws exception otherwise
        [[nodiscard]] std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> findEdge(
            const std::shared_ptr<const EuclideanGraphConfiguration>& a,
            const std::shared_ptr<const EuclideanGraphConfiguration>& b) const;

        //! \returns The edge in the graph that contains \p a and \p b if it exists, throws exception otherwise
        [[nodiscard]] std::shared_ptr<UndirectedGraphEdge<EuclideanGraphConfiguration>> findEdge(unsigned int a,
                                                                                                 unsigned int b) const;

        //! \copydoc EnvironmentBase
        [[nodiscard]] float longestPath() const final override;

       protected:
        //! \copydoc PointGraphEnvironmentBase
        void internalFromJson(const nlohmann::json& j) final override;

        friend class SampledEuclideanGraphEnvironment;
        friend void from_json(const nlohmann::json& j, EuclideanGraphEnvironment& e);
    };

    void from_json(const nlohmann::json& j, EuclideanGraphEnvironment& e);
}  // namespace grstapse