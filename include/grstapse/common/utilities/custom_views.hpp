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

// region Includes
// Global
#include <functional>
#include <memory>
#include <numeric>
#include <vector>
// External
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/ref.hpp>
#include <range/v3/view/transform.hpp>
// Local
#include "grstapse/common/utilities/custom_concepts.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/task.hpp"
// endregion

/*!
 * \brief This file contains a number of specific view typedefs as well as some utility functions to extend the
 * capabilities of ranges and views
 *
 * \file views_extension.hpp
 */

namespace grstapse
{
    // Note: clion does not seem to understand views, so there are a lot of red squiggly lines even though it compiles
    //! A view used to iterate through a set of robots that are assigned to either a task or transition
    using CoalitionView = ::ranges::transform_view<
        ::ranges::filter_view<::ranges::iota_view<unsigned int, unsigned int>, std::function<bool(unsigned int)>>,
        std::function<const std::shared_ptr<const Robot>&(unsigned int)>>;

    //! A view used to iterate through tasks in a plan
    using PlanView = ::ranges::transform_view<::ranges::ref_view<const std::vector<unsigned int>>,
                                              std::function<const std::shared_ptr<const Task>&(unsigned int)>>;
}  // namespace grstapse