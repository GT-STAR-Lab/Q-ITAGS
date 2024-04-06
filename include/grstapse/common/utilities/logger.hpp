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

// External
#include <fmt/format.h>
#include <spdlog/spdlog.h>

namespace grstapse
{
    /*!
     * \brief Wrapper for spdlog that sets up a logger that writes
     *        both to the console and a file
     */
    class Logger
    {
       public:
        /*!
         * \brief Writes a message to the logger using fmt format with
         *        the debug level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void debug(fmt::format_string<Args...> message, Args&&... args)
        {
            instance()->debug(message, std::forward<Args>(args)...);
        }

        /*!
         * \brief Writes a formatted message to the logger with
         *        the debug level
         *
         * \param message The formatted log message
         */
        static void debug(const std::string_view& message);

        /*!
         * \brief Writes a message to the logger using fmt format with
         *        the info level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void info(fmt::format_string<Args...> message, Args&&... args)
        {
            instance()->info(message, std::forward<Args>(args)...);
        }

        /*!
         * \brief Writes a formatted message to the logger with
         *        the info level
         *
         * \param message The formatted log message
         */
        static void info(const std::string_view& message);

        /*!
         * \brief Writes a message to the logger using fmt format with
         *        the warn level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void warn(fmt::format_string<Args...> message, Args&&... args)
        {
            instance()->warn(message, std::forward<Args>(args)...);
        }

        /*!
         * \brief Writes a formatted message to the logger with
         *        the warn level
         *
         * \param message The formatted log message
         */
        static void warn(const std::string_view& message);

        /*!
         * \brief Writes a message to the logger using fmt format with
         *        the error level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        static void error(fmt::format_string<Args...> message, Args&&... args)
        {
            instance()->error(message, std::forward<Args>(args)...);
            instance()->flush();
        }

        /*!
         * \brief Writes a formatted message to the logger with
         *        the error level
         *
         * \param message The formatted log message
         */
        static void error(const std::string_view& message);

        /*!
         * \brief Writes a message to the logger using fmt format with
         *        the critical level
         *
         * \param message The format string for the log message
         * \param args The arguments to be format into the log message
         *
         * \tparam Args A list of argument types to formatted into the
         *              log message
         */
        template <typename... Args>
        [[maybe_unused]] static void critical(fmt::format_string<Args...> message, Args&&... args)
        {
            instance()->critical(message, std::forward<Args>(args)...);
            instance()->flush();
        }

        /*!
         * \brief Writes a formatted message to the logger with
         *        the critical level
         *
         * \param message The formatted log message
         */
        [[maybe_unused]] static void critical(const std::string_view& message);

        //! Flushes the buffer
        static void flush();

       private:
        //! \returns the logger as a singleton
        static std::shared_ptr<spdlog::logger> instance();

        //! \brief Hides the constructor from the user
        Logger() = default;
    };

}  // namespace grstapse