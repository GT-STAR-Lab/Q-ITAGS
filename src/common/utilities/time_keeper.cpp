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
#include "grstapse/common/utilities/time_keeper.hpp"

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/logger.hpp"

namespace grstapse
{
    TimeKeeper& TimeKeeper::instance()
    {
        static TimeKeeper singleton;
        return singleton;
    }

    void TimeKeeper::setActive(const std::string& timer_name, Timer* timer)
    {
        if(m_currently_active_timers.contains(timer_name))
        {
            auto [first, last] = m_currently_active_timers.equal_range(timer_name);
            for(auto iter = first; iter != last; ++iter)
            {
                if(timer == iter->second)
                {
                    Logger::warn("Timer '{0:s}' already active", timer_name);
                    return;
                }
            }
        }

        m_currently_active_timers.emplace(timer_name, timer);
    }

    void TimeKeeper::setInactive(const std::string& timer_name, Timer* timer)
    {
        if(m_currently_active_timers.contains(timer_name))
        {
            auto [first, last] = m_currently_active_timers.equal_range(timer_name);
            for(auto iter = first; iter != last; ++iter)
            {
                if(timer == iter->second)
                {
                    m_currently_active_timers.erase(iter);
                    return;
                }
            }
        }

        Logger::warn("Timer '{0:s}' not active", timer_name);
    }

    void TimeKeeper::reset(const std::string& timer_name)
    {
        if(not m_times.contains(timer_name))
        {
            throw createLogicError(fmt::format("Request for reset of unknown timer '{0:s}'", timer_name));
        }
        m_times[timer_name] = 0;
    }

    void TimeKeeper::resetAll()
    {
        if(not m_currently_active_timers.empty())
        {
            throw createLogicError("Cannot reset all recorded times while there are still active timers");
        }
        for(auto& [name, timer]: m_times)
        {
            timer = 0;
        }
    }

    void TimeKeeper::remove(const std::string& timer_name)
    {
        if(not m_times.contains(timer_name))
        {
            throw createLogicError(fmt::format("Request for removal of unknown timer '{0:s}'", timer_name));
        }
        m_times.erase(timer_name);
    }

    void TimeKeeper::removeAll()
    {
        if(not m_currently_active_timers.empty())
        {
            throw createLogicError("Cannot remove all recorded times while there are still active timers");
        }
        m_times.clear();
    }

    float TimeKeeper::time(const std::string& timer_name) const
    {
        if(not m_times.contains(timer_name) and not m_currently_active_timers.contains(timer_name))
        {
            throw createLogicError(fmt::format("Request for time from unknown timer '{0:s}'", timer_name));
        }
        float rv = 0.0;
        if(m_times.contains(timer_name))
        {
            rv += m_times.at(timer_name);
        }
        if(m_currently_active_timers.contains(timer_name))
        {
            auto [first, last] = m_currently_active_timers.equal_range(timer_name);
            for(auto iter = first; iter != last; ++iter)
            {
                rv += iter->second->get();
            }
        }
        return rv;
    }

    void TimeKeeper::increment(const std::string& timer_name, float amount)
    {
        if(not m_times.contains(timer_name))
        {
            m_times[timer_name] = 0;
        }
        m_times[timer_name] += amount;
    }

}  // namespace grstapse