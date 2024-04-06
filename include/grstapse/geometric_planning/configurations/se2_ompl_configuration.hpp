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
#include "grstapse/geometric_planning/configurations/ompl_configuration.hpp"

namespace grstapse
{
    // Forward Declaration
    class Se2StateOmplConfiguration;
    enum class OmplGoalType : uint8_t;

    /*!
     * \brief A task configuration from SE(2) that works with an ompl motion planner
     *
     * \note This class is a base class for the possible extension of configuration base to OMPL set of states and
     *       OMPL goal space
     */
    class Se2OmplConfiguration : public OmplConfiguration
    {
       protected:
        //! Constructor
        explicit Se2OmplConfiguration(OmplGoalType goal_type);
    };

}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const grstapse::Se2OmplConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const grstapse::Se2OmplConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const grstapse::Se2OmplConfiguration>& c);
    };

    template <>
    struct adl_serializer<std::shared_ptr<grstapse::Se2OmplConfiguration>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<grstapse::Se2OmplConfiguration> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<grstapse::Se2OmplConfiguration>& c);
    };
}  // namespace nlohmann