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
#include "grstapse/task_allocation/species_task_failure.hpp"

namespace grstapse
{
    SpeciesTaskFailure::SpeciesTaskFailure(const detail::SpeciesTaskFailureParametersImpl& parameters)
        : species(parameters.species)
        , task(parameters.task)
    {}

    SpeciesTaskFailure::SpeciesTaskFailure(detail::SpeciesTaskFailureParametersImpl&& parameters)
        : species(std::move(parameters.species))
        , task(std::move(parameters.task))
    {}

    SpeciesTaskFailure::SpeciesTaskFailure(const std::string& species, unsigned int task)
        : species(species)
        , task(task)
    {}

    SpeciesTaskFailure::SpeciesTaskFailure(std::string&& species, unsigned int task)
        : species(std::move(species))
        , task(task)
    {}
}  // namespace grstapse