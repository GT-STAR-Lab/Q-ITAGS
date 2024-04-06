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
#include <memory>
// Local
#include "grstapse/task_allocation/itags/itags_builder_options.hpp"

namespace grstapse
{
    // Forward Declaration
    class Itags;
    class ItagsProblemInputs;

    /*!
     * \brief Builds an ITAGS shared_ptr
     *
     * \see Itags
     */
    class ItagsBuilder
    {
       public:
        //! Constructor
        explicit ItagsBuilder(const ItagsBuilderOptions& builder_options = ItagsBuilderOptions());

        //! \returns An ITAGS shared_ptr
        [[nodiscard]] std::shared_ptr<Itags> build(
            const std::shared_ptr<const ItagsProblemInputs>& problem_inputs) const;

       private:
        ItagsBuilderOptions m_builder_options;
    };

}  // namespace grstapse