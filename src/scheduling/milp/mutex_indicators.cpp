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
#include "grstapse/scheduling/milp/mutex_indicators.hpp"

// External
#include <fmt/format.h>
#include <fmt/ranges.h>
// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/ms_name_scheme_base.hpp"

namespace grstapse
{
    MutexIndicators::MutexIndicators(const std::set<std::pair<unsigned int, unsigned int>>& mutex_constraints,
                                     const std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints,
                                     const std::shared_ptr<const MsNameSchemeBase>& name_scheme,
                                     bool master)
        : m_precedence_constraints(precedence_constraints)
        , m_name_scheme(name_scheme)
        , m_master(master)
    {
        for(const auto& p: mutex_constraints)
        {
            if(m_precedence_constraints.contains(p) || m_precedence_constraints.contains({p.second, p.first}))
            {
                continue;
            }

            // Give an empty GRBVar for now
            m_indicators[p] = GRBVar();
        }
    }

    MutexIndicators::MutexIndicators(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs,
                                     const std::shared_ptr<const MsNameSchemeBase>& name_scheme,
                                     bool master)
        : MutexIndicators(problem_inputs->mutexConstraints(),
                          problem_inputs->precedenceConstraints(),
                          name_scheme,
                          master)
    {}

    void MutexIndicators::createVariables(GRBModel& model)
    {
        if(m_master)
        {
            // Create mutex indicators
            for(auto& [k, v]: m_indicators)
            {
                v = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, m_name_scheme->createMutexIndicatorName(k.first, k.second));
            }
        }
        else
        {
            // Create mutex indicators
            for(auto& [k, v]: m_indicators)
            {
                v = model.addVar(0.0,
                                 1.0,
                                 0.0,
                                 GRB_CONTINUOUS,
                                 m_name_scheme->createMutexIndicatorName(k.first, k.second));
            }
        }
    }

    std::vector<std::pair<unsigned int, unsigned int>> MutexIndicators::precedenceSet() const
    {
        std::vector<std::pair<unsigned int, unsigned int>> rv;
        rv.reserve(m_indicators.size());
        for(const auto& [p, indicator]: m_indicators)
        {
            if(indicator.get(GRB_DoubleAttr_X) > 0.5)
            {
                rv.emplace_back(p);
            }
            else
            {
                rv.emplace_back(p.second, p.first);
            }
        }
        return rv;
    }

    GRBVar& MutexIndicators::get(const std::pair<unsigned int, unsigned int>& p)
    {
        if(m_indicators.contains(p))
        {
            return m_indicators[p];
        }
        throw createLogicError(fmt::format("Cannot find indicator for {}", p));
    }
}  // namespace grstapse