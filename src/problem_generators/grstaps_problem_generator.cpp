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
#include "grstapse/problem_generators/grstaps_problem_generator.hpp"

// Global
#include <fstream>
// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    nlohmann::json GrstapsProblemGenerator::generate() const
    {
        nlohmann::json j;

        j[constants::k_motion_planners] = generateMotionPlanners();
        j[constants::k_species]         = generateSpecies();
        j[constants::k_robots]          = generateRobots();
        j[constants::k_tasks]           = generateTasks();

        // Parameters
        j[constants::k_fcpop_parameters]     = generateFcpopParameters();
        j[constants::k_itags_parameters]     = generateItagsParameters();
        j[constants::k_scheduler_parameters] = generateSchedulerParameters();

        return j;
    }
    void GrstapsProblemGenerator::writeToFile(const std::string& filepath) const
    {
        std::ofstream fout(filepath);
        nlohmann::json j = generate();
        fout << j;
    }
}  // namespace grstapse