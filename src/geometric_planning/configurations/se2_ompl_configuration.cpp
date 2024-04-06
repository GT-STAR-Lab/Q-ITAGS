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
#include "grstapse/geometric_planning/configurations/se2_ompl_configuration.hpp"

#include <ompl/base/ScopedState.h>

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/se2_state_ompl_configuration.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    Se2OmplConfiguration::Se2OmplConfiguration(OmplGoalType goal_type)
        : OmplConfiguration(goal_type, OmplStateSpaceType::e_se2)
    {}
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<const grstapse::Se2OmplConfiguration>
    adl_serializer<std::shared_ptr<const grstapse::Se2OmplConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<grstapse::Se2OmplConfiguration>>::from_json(j);
    }

    std::shared_ptr<grstapse::Se2OmplConfiguration>
    adl_serializer<std::shared_ptr<grstapse::Se2OmplConfiguration>>::from_json(const json& j)
    {
        grstapse::json_ext::validateJson(j, {{grstapse::constants::k_goal_type, nlohmann::json::value_t::string}});
        const grstapse::OmplGoalType ompl_goal_type = j.at(grstapse::constants::k_goal_type);
        switch(ompl_goal_type)
        {
            case grstapse::OmplGoalType::e_state:
            {
                return j.get<std::shared_ptr<grstapse::Se2StateOmplConfiguration>>();
            }
            case grstapse::OmplGoalType::e_set_of_states:
            {
                throw grstapse::createLogicError("Not implemented");
            }
            case grstapse::OmplGoalType::e_space:
            {
                throw grstapse::createLogicError("Not implemented");
            }
            default:
            {
                throw grstapse::createLogicError(
                    fmt::format("Unknown OmplGoalType: {0:s}",
                                j.at(grstapse::constants::k_goal_type).get<std::string>()));
            }
        }
    }

    void adl_serializer<std::shared_ptr<const grstapse::Se2OmplConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<const grstapse::Se2OmplConfiguration>& c)
    {
        switch(c->goalType())
        {
            case grstapse::OmplGoalType::e_state:
            {
                j = std::dynamic_pointer_cast<const grstapse::Se2StateOmplConfiguration>(c);
                return;
            }
            case grstapse::OmplGoalType::e_set_of_states:
            {
                throw grstapse::createLogicError("Not implemented");
            }
            case grstapse::OmplGoalType::e_space:
            {
                throw grstapse::createLogicError("Not implemented");
            }
            default:
            {
                throw grstapse::createLogicError("Unknown OmplGoalType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<grstapse::Se2OmplConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<grstapse::Se2OmplConfiguration>& c)
    {
        adl_serializer<std::shared_ptr<const grstapse::Se2OmplConfiguration>>::to_json(j, c);
    }

}  // namespace nlohmann