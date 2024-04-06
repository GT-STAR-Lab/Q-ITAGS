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
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"

namespace grstapse
{
    // Forward Declarations
    enum class GraphType : uint8_t;

    //! Configuration from a graph
    class GraphConfiguration : public ConfigurationBase
    {
       public:
        //! \returns The id of the graph vertex
        [[nodiscard]] inline unsigned int id() const;

        //! \returns The type of graph
        [[nodiscard]] inline GraphType graphType() const;

       protected:
        //! Constructor
        explicit GraphConfiguration(GraphType graph_type, unsigned int id);

        GraphType m_graph_type;
        unsigned int m_id;
    };

    // Inline Function
    unsigned int GraphConfiguration::id() const
    {
        return m_id;
    }

    GraphType GraphConfiguration::graphType() const
    {
        return m_graph_type;
    }

}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const grstapse::GraphConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const grstapse::GraphConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const grstapse::GraphConfiguration>& c);
    };

    template <>
    struct adl_serializer<std::shared_ptr<grstapse::GraphConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::GraphConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<grstapse::GraphConfiguration>& c);
    };
}  // namespace nlohmann