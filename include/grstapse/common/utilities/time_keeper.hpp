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
#include <string>
#include <unordered_map>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/timer.hpp"

namespace grstapse
{
    /*!
     * \brief Global singleton that stores the times for named timers
     *
     * \see Timer
     */
    class TimeKeeper : public Noncopyable
    {
       public:
        //! \returns The singleton instance of this class
        static TimeKeeper& instance();

        //!
        void setActive(const std::string& timer_name, Timer* timer);

        //!
        void setInactive(const std::string& timer_name, Timer* timer);

        //! Resets a timer of name \p timer_name
        void reset(const std::string& timer_name);

        //! Reset all the timers
        void resetAll();

        //! Removes a timer of name \p timer_name
        void remove(const std::string& timer_name);

        //! Removes all the timers
        void removeAll();

        //! \returns The value for a named timer
        [[nodiscard]] float time(const std::string& timer_name) const;

        void increment(const std::string& timer_name, float amount);

       private:
        //! Constructor
        TimeKeeper() = default;
        std::unordered_map<std::string, float> m_times;
        std::unordered_multimap<std::string, Timer*> m_currently_active_timers;
    };

}  // namespace grstapse