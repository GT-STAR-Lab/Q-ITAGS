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
#include <concepts>
// External
#include <gurobi_c++.h>

/*!
 * \file concepts_extension.hpp
 *
 * \brief This file contains a number of custom concepts
 */

namespace grstapse
{
    // region milp dual cut
    template <typename T>
    struct dual_cut_return_type : std::false_type
    {};

    template <>
    struct dual_cut_return_type<float> : std::true_type
    {};

    template <>
    struct dual_cut_return_type<double> : std::true_type
    {};

    template <>
    struct dual_cut_return_type<GRBLinExpr> : std::true_type
    {};

    template <typename T>
    concept DualCutReturnType = dual_cut_return_type<T>::value;

    template <typename T>
    struct dual_cut_variable_type : std::false_type
    {};

    template <>
    struct dual_cut_variable_type<float> : std::true_type
    {};

    template <>
    struct dual_cut_variable_type<double> : std::true_type
    {};

    template <>
    struct dual_cut_variable_type<GRBVar> : std::true_type
    {};

    template <typename T>
    concept DualCutVariableType = dual_cut_variable_type<T>::value;
    // endregion
}  // namespace grstapse