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
#include "grstapse/common/utilities/logger.hpp"

// Global
#include <chrono>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>
// External
#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
// Local
#include "grstapse/config.hpp"

namespace grstapse
{
    std::shared_ptr<spdlog::logger> Logger::instance()
    {
        // Only initialize once
        static bool first = true;
        if(first)
        {
            first = false;

            // Create the logs folder if it does not exist
            std::filesystem::path logs_folder = "logs";
            if(!std::filesystem::exists(logs_folder))
            {
                std::filesystem::create_directory(logs_folder);
            }

            // Create logger sinks
            std::vector<spdlog::sink_ptr> sinks;
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
            sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                fmt::format("logs/{:%m-%d-%Y.%Hh-%Mm-%Ss}.txt", fmt::localtime(std::chrono::system_clock::now()))));

            // Configure a logger
            spdlog::set_default_logger(
                std::make_shared<spdlog::logger>(std::string(s_project_name), begin(sinks), end(sinks)));
            spdlog::set_pattern("%^[%D %H:%M:%S.%e] [%n] [thread %t] [%l] %v%$");
            spdlog::set_level(spdlog::level::debug);
            spdlog::flush_on(spdlog::level::err);
        }

        return spdlog::default_logger();
    }

    void Logger::debug(const std::string_view& message)
    {
        instance()->debug(message);
    }

    void Logger::info(const std::string_view& message)
    {
        instance()->info(message);
    }

    void Logger::warn(const std::string_view& message)
    {
        instance()->warn(message);
    }

    void Logger::error(const std::string_view& message)
    {
        instance()->error(message);
        instance()->flush();
    }

    [[maybe_unused]] void Logger::critical(const std::string_view& message)
    {
        instance()->critical(message);
        instance()->flush();
    }

    void Logger::flush()
    {
        instance()->flush();
    }
}  // namespace grstapse