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
#include <functional>
#include <optional>
#include <string>
// Local
#include "grstapse/common/utilities/timer.hpp"

namespace grstapse
{
    /*!
     * \brief A runner for the Timer
     *
     * \see Timer
     */
    class TimerRunner
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param timer The timer to use
         * \param name The name of the timer
         */
        TimerRunner(const std::string& name);

        //! Destructor
        ~TimerRunner();

       private:
        Timer m_timer;
        std::string m_name;
    };

    /*!
     * \brief Call function and record the time it takes to run
     *
     * \param timer_name The name of the timer
     * \param f A callable
     */
    void doTimed(const std::string& timer_name, std::invocable auto&& f)
    {
        TimerRunner timer_runner(timer_name);
        f();
    }

}  // namespace grstapse