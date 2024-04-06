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
#include "grstapse/geometric_planning/configurations/graph_configuration.hpp"

#include <grstapse/geometric_planning/query_results/euclidean_graph_motion_planner_query_result.hpp>

namespace grstapse
{
    //! Configuration from a graph that represents a 2D point
    class EuclideanGraphConfiguration : public GraphConfiguration
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param id The identifier for the graph vertex
         * \param x The x coordinate of this point
         * \param y The y coordinate of this point
         */
        EuclideanGraphConfiguration(unsigned int id, float x, float y);

        //! \returns The x coordinate of this point
        [[nodiscard]] inline float x() const;

        //! \returns The y coordinate of this point
        [[nodiscard]] inline float y() const;

        //! \copydoc ConfigurationBase
        [[nodiscard]] float euclideanDistance(const ConfigurationBase& rhs) const final override;

        //! Euclidean Distance
        [[nodiscard]] float euclideanDistance(const EuclideanGraphConfiguration& rhs) const;

        //! Equality operator
        [[nodiscard]] bool operator==(const ConfigurationBase& rhs) const final override;

        //! Equality operator
        [[nodiscard]] bool operator==(const EuclideanGraphConfiguration& rhs) const;

       private:
        float m_x;
        float m_y;
    };

    void to_json(nlohmann::json& j, const EuclideanGraphConfiguration& c);

    // Inline Function
    float EuclideanGraphConfiguration::x() const
    {
        return m_x;
    }

    float EuclideanGraphConfiguration::y() const
    {
        return m_y;
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const grstapse::EuclideanGraphConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const grstapse::EuclideanGraphConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const grstapse::EuclideanGraphConfiguration>& c);
    };

    template <>
    struct adl_serializer<std::shared_ptr<grstapse::EuclideanGraphConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::EuclideanGraphConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<grstapse::EuclideanGraphConfiguration>& c);
    };
}  // namespace nlohmann