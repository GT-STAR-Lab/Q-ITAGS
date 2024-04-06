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
#include "grstapse/geometric_planning/environments/graph_environment.hpp"

namespace grstapse
{
    // Forward Declarations
    enum class EuclideanGraphType : uint8_t;

    /*!
     * \brief Base class for euclidean graph environments
     */
    class EuclideanGraphEnvironmentBase : public GraphEnvironment
    {
       public:
        //! \returns The euclidean graph type
        [[nodiscard]] inline EuclideanGraphType euclideanGraphType() const;

        //! \returns Whether the graph is a completely connected graph
        [[nodiscard]] inline bool isComplete() const;

       protected:
        //! Constructor
        EuclideanGraphEnvironmentBase(EuclideanGraphType point_graph_type, bool is_complete);

        /*!
         * \brief Internal handler for deserializing from json
         *
         * We allow the option of loading the json for a point graph environment directly or
         * indirectly through a "graph_file" json element. The main adl_serializer::from_json functions
         * will handle determining which it is an will call this function with the actual point graph
         * environment json
         *
         * \returna A PointGraphEnvironmentBase
         */
        virtual void internalFromJson(const nlohmann::json& j) = 0;

        EuclideanGraphType m_point_graph_type;
        bool m_is_complete;
    };

    // Inline Functions
    EuclideanGraphType EuclideanGraphEnvironmentBase::euclideanGraphType() const
    {
        return m_point_graph_type;
    }

    bool EuclideanGraphEnvironmentBase::isComplete() const
    {
        return m_is_complete;
    }

}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<grstapse::EuclideanGraphEnvironmentBase>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::EuclideanGraphEnvironmentBase> from_json(const json& j);
    };
}  // namespace nlohmann