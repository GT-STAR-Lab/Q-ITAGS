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
#include "grstapse/geometric_planning/environments/euclidean_graph_environment_base.hpp"

namespace grstapse
{
    // Forward Declarations
    class EuclideanGraphEnvironment;

    /*!
     * \brief Environment that represents an individual euclidean graph with sampled edge probabilities
     *
     * Each internal graph contains the same vertices. The edges may be different depending on the
     * edge probabilities.
     */
    class SampledEuclideanGraphEnvironment : public EuclideanGraphEnvironmentBase
    {
       public:
        //! Default Constructor
        SampledEuclideanGraphEnvironment();

        //! Constructor
        explicit SampledEuclideanGraphEnvironment(bool is_complete);

        //! Adds one of the sampled graphs
        void addGraph(const std::shared_ptr<EuclideanGraphEnvironment>& g);

        [[nodiscard]] inline const std::vector<std::shared_ptr<EuclideanGraphEnvironment>>& graphs() const;

        //! \returns The \p index'th graph environment
        [[nodiscard]] virtual const std::shared_ptr<EuclideanGraphEnvironment>& graph(unsigned int index) const;

        //! \returns The number of sampled graphs
        [[nodiscard]] virtual inline unsigned int numGraphs() const;

        //! \copydoc EnvironmentBase
        [[nodiscard]] float longestPath() const override;

       protected:
        //! \copydoc PointGraphEnvironmentBase
        void internalFromJson(const nlohmann::json& j) final override;

       protected:
        std::vector<std::shared_ptr<EuclideanGraphEnvironment>> m_graphs;

        friend void from_json(const nlohmann::json& j, SampledEuclideanGraphEnvironment& e);
    };

    //! Deserialize from json
    void from_json(const nlohmann::json& j, SampledEuclideanGraphEnvironment& environment);

    // Inline functions
    const std::vector<std::shared_ptr<EuclideanGraphEnvironment>>& SampledEuclideanGraphEnvironment::graphs() const
    {
        return m_graphs;
    }

    unsigned int SampledEuclideanGraphEnvironment::numGraphs() const
    {
        return m_graphs.size();
    }

}  // namespace grstapse