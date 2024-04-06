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
#include "grstapse/task_allocation/species_task_pair_failure.hpp"

namespace grstapse
{
    SpeciesTaskPairFailure::SpeciesTaskPairFailure(const detail::SpeciesTaskPairFailureParametersImpl& parameters)
        : species(parameters.species)
        , predecessor_task_index(parameters.predecessor_task_index)
        , successor_task_index(parameters.successor_task_index)
    {}

    SpeciesTaskPairFailure::SpeciesTaskPairFailure(detail::SpeciesTaskPairFailureParametersImpl&& parameters)
        : species(std::move(parameters.species))
        , predecessor_task_index(std::move(parameters.predecessor_task_index))
        , successor_task_index(std::move(parameters.successor_task_index))
    {}

    SpeciesTaskPairFailure::SpeciesTaskPairFailure(const std::string& species,
                                                   unsigned int predecessor_task_index,
                                                   unsigned int successor_task_index)
        : species(species)
        , predecessor_task_index(predecessor_task_index)
        , successor_task_index(successor_task_index)
    {}

    SpeciesTaskPairFailure::SpeciesTaskPairFailure(std::string&& species,
                                                   unsigned int predecessor_task_index,
                                                   unsigned int successor_task_index)
        : species(std::move(species))
        , predecessor_task_index(predecessor_task_index)
        , successor_task_index(successor_task_index)
    {}
}  // namespace grstapse