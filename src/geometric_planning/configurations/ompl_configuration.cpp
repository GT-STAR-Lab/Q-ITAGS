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
#include "grstapse/geometric_planning/configurations/ompl_configuration.hpp"

// External
#include <fmt/format.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/configurations/se2_ompl_configuration.hpp"
#include "grstapse/geometric_planning/configurations/se3_ompl_configuration.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    OmplConfiguration::OmplConfiguration(OmplGoalType goal_type, OmplStateSpaceType state_space_type)
        : ConfigurationBase(ConfigurationType::e_ompl)
        , m_ompl_goal_type(goal_type)
        , m_ompl_state_space_type(state_space_type)
    {}
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<const grstapse::OmplConfiguration>
    adl_serializer<std::shared_ptr<const grstapse::OmplConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<grstapse::OmplConfiguration>>::from_json(j);
    }

    std::shared_ptr<grstapse::OmplConfiguration>
    adl_serializer<std::shared_ptr<grstapse::OmplConfiguration>>::from_json(const json& j)
    {
        grstapse::json_ext::validateJson(j,
                                         {{grstapse::constants::k_state_space_type, nlohmann::json::value_t::string}});
        const grstapse::OmplStateSpaceType ompl_state_space_type = j.at(grstapse::constants::k_state_space_type);

        switch(ompl_state_space_type)
        {
            case grstapse::OmplStateSpaceType::e_se2:
            {
                return j.get<std::shared_ptr<grstapse::Se2OmplConfiguration>>();
            }
            case grstapse::OmplStateSpaceType::e_se3:
            {
                return j.get<std::shared_ptr<grstapse::Se3OmplConfiguration>>();
            }
            default:
            {
                throw grstapse::createLogicError(
                    fmt::format("Unknown OmplStateSpaceType: {0:s}",
                                j.at(grstapse::constants::k_state_space_type).get<std::string>()));
            }
        }
    }

    void adl_serializer<std::shared_ptr<const grstapse::OmplConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<const grstapse::OmplConfiguration>& c)
    {
        switch(c->stateSpaceType())
        {
            case grstapse::OmplStateSpaceType::e_se2:
            {
                j = std::dynamic_pointer_cast<const grstapse::Se2OmplConfiguration>(c);
                return;
            }
            case grstapse::OmplStateSpaceType::e_se3:
            {
                j = std::dynamic_pointer_cast<const grstapse::Se3OmplConfiguration>(c);
                return;
            }
            default:
            {
                throw grstapse::createLogicError("Unknown OmplStateSpaceType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<grstapse::OmplConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<grstapse::OmplConfiguration>& c)
    {
        adl_serializer<std::shared_ptr<const grstapse::OmplConfiguration>>::to_json(j, c);
    }
}  // namespace nlohmann