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
#include <nlohmann/json.hpp>
// Local
#include "grstapse/problem_generators/problem_generator.hpp"

namespace grstapse
{
    /*!
     * Base class for generating GRSTAPS problems
     */
    class GrstapsProblemGenerator : public ProblemGenerator
    {
       public:
        [[nodiscard]] virtual nlohmann::json generate() const;
        void writeToFile(const std::string& filepath) const;

       protected:
        [[nodiscard]] virtual nlohmann::json generateTasks() const   = 0;
        [[nodiscard]] virtual nlohmann::json generateRobots() const  = 0;
        [[nodiscard]] virtual nlohmann::json generateSpecies() const = 0;

        [[nodiscard]] virtual nlohmann::json generateFcpopParameters() const     = 0;
        [[nodiscard]] virtual nlohmann::json generateItagsParameters() const     = 0;
        [[nodiscard]] virtual nlohmann::json generateSchedulerParameters() const = 0;
        [[nodiscard]] virtual nlohmann::json generateMotionPlanners() const      = 0;
    };

}  // namespace grstapse