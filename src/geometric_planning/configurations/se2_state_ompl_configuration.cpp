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
#include "grstapse/geometric_planning/configurations/se2_state_ompl_configuration.hpp"

// Global
#include <cmath>
// External
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/geometric_planning/motion_planning_enums.hpp"

namespace grstapse
{
    Se2StateOmplConfiguration::Se2StateOmplConfiguration(const float x, const float y, const float yaw)
        : Se2OmplConfiguration(OmplGoalType::e_state)
        , m_x(x)
        , m_y(y)
        , m_yaw(yaw)
    {}

    bool Se2StateOmplConfiguration::operator==(const ConfigurationBase& rhs) const
    {
        if(m_configuration_type != rhs.configurationType())
        {
            return false;
        }

        const auto* rhs_ompl = dynamic_cast<const OmplConfiguration*>(&rhs);
        if(!rhs_ompl || m_ompl_state_space_type != rhs_ompl->stateSpaceType() ||
           m_ompl_goal_type != rhs_ompl->goalType())
        {
            return false;
        }

        const auto* rhs_ompl_se2_state = dynamic_cast<const Se2StateOmplConfiguration*>(rhs_ompl);
        if(!rhs_ompl_se2_state)
        {
            return false;
        }

        return *this == *rhs_ompl_se2_state;
    }

    bool Se2StateOmplConfiguration::operator==(const Se2StateOmplConfiguration& rhs) const
    {
        return m_x == rhs.m_x && m_y == rhs.m_y && m_yaw == rhs.m_yaw;
    }

    float Se2StateOmplConfiguration::euclideanDistance(const ConfigurationBase& rhs) const
    {
        if(m_configuration_type != rhs.configurationType())
        {
            throw createLogicError("Cannot compute the euclidean distance for two configurations of different types");
        }

        const auto* rhs_ompl = dynamic_cast<const OmplConfiguration*>(&rhs);
        if(!rhs_ompl || m_ompl_state_space_type != rhs_ompl->stateSpaceType() ||
           m_ompl_goal_type != rhs_ompl->goalType())
        {
            throw createLogicError("'rhs' claims it is a OmplConfiguration, but it is not");
        }

        const auto* rhs_ompl_se2_state = dynamic_cast<const Se2StateOmplConfiguration*>(rhs_ompl);
        if(!rhs_ompl_se2_state)
        {
            throw createLogicError("'rhs' claims it is a Se2StateOmplConfiguration, but it is not");
        }

        return euclideanDistance(*rhs_ompl_se2_state);
    }

    float Se2StateOmplConfiguration::euclideanDistance(const Se2StateOmplConfiguration& rhs) const
    {
        const float x_diff = m_x - rhs.m_x;
        const float y_diff = m_y - rhs.m_y;
        return std::sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    }

    ompl::base::ScopedStatePtr Se2StateOmplConfiguration::convertToScopedStatePtr(
        const ompl::base::StateSpacePtr& state_space) const
    {
        auto rv     = std::make_shared<ompl::base::ScopedState<>>(state_space);
        auto* state = rv->get()->as<ompl::base::SE2StateSpace::StateType>();
        state->setX(m_x);
        state->setY(m_y);
        state->setYaw(m_yaw);
        return rv;
    }

    ompl::base::GoalPtr Se2StateOmplConfiguration::convertToGoalPtr(
        const ompl::base::SpaceInformationPtr& space_information) const
    {
        auto rv     = std::make_shared<ompl::base::GoalState>(space_information);
        auto* state = space_information->allocState()->as<ompl::base::SE2StateSpace::StateType>();
        state->setX(m_x);
        state->setY(m_y);
        state->setYaw(m_yaw);
        rv->setState(state);
        return rv;
    }
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<const grstapse::Se2StateOmplConfiguration>
    adl_serializer<std::shared_ptr<const grstapse::Se2StateOmplConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<grstapse::Se2StateOmplConfiguration>>::from_json(j);
    }

    std::shared_ptr<grstapse::Se2StateOmplConfiguration>
    adl_serializer<std::shared_ptr<grstapse::Se2StateOmplConfiguration>>::from_json(const json& j)
    {
        grstapse::json_ext::validateJson(j,
                                         {{grstapse::constants::k_x, nlohmann::json::value_t::number_float},
                                          {grstapse::constants::k_y, nlohmann::json::value_t::number_float},
                                          {grstapse::constants::k_yaw, nlohmann::json::value_t::number_float}});

        const float x   = j.at(grstapse::constants::k_x);
        const float y   = j.at(grstapse::constants::k_y);
        const float yaw = j.at(grstapse::constants::k_yaw);

        return std::make_shared<grstapse::Se2StateOmplConfiguration>(x, y, yaw);
    }

    void adl_serializer<std::shared_ptr<const grstapse::Se2StateOmplConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<const grstapse::Se2StateOmplConfiguration>& c)
    {
        j[grstapse::constants::k_configuration_type] = c->configurationType();
        j[grstapse::constants::k_state_space_type]   = c->stateSpaceType();
        j[grstapse::constants::k_goal_type]          = c->goalType();
        j[grstapse::constants::k_x]                  = c->x();
        j[grstapse::constants::k_y]                  = c->y();
        j[grstapse::constants::k_yaw]                = c->yaw();
    }

    void adl_serializer<std::shared_ptr<grstapse::Se2StateOmplConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<grstapse::Se2StateOmplConfiguration>& c)
    {
        adl_serializer<std::shared_ptr<grstapse::Se2StateOmplConfiguration>>::to_json(j, c);
    }
}  // namespace nlohmann