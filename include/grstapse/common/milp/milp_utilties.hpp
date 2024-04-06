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
#include <gurobi_c++.h>

namespace grstapse
{
    //! \returns The value from a MILP variable
    [[nodiscard]] inline double variableValue(const GRBVar& var)
    {
        return var.get(GRB_DoubleAttr_X);
    }
    //! \returns The value from a MILP variable
    [[nodiscard]] inline double variableValue(const GRBVar&& var)
    {
        return var.get(GRB_DoubleAttr_X);
    }
    //! \returns The value from a MILP variable
    [[nodiscard]] inline double variableValue(GRBModel& model, const std::string& variable_name)
    {
        return variableValue(model.getVarByName(variable_name));
    }
    //! \returns The value from a MILP variable
    [[nodiscard]] inline double variableValue(const std::shared_ptr<GRBModel>& model, const std::string& variable_name)
    {
        return variableValue(model->getVarByName(variable_name));
    }
    //! Fixes a MILP variable to be a set \p value
    inline void fixVariable(GRBVar& var, double value)
    {
        var.set(GRB_DoubleAttr_LB, value);
        var.set(GRB_DoubleAttr_UB, value);
    }
    //! Fixes a MILP variable to be a set \p value
    inline void fixVariable(GRBVar&& var, double value)
    {
        var.set(GRB_DoubleAttr_LB, value);
        var.set(GRB_DoubleAttr_UB, value);
    }
    //! Fixes a MILP variable to be a set \p value
    inline void fixVariable(GRBModel& model, const std::string& variable_name, double value)
    {
        fixVariable(model.getVarByName(variable_name), value);
    }
    /*!
     * \returns The value of the dual variable for the specific constraint
     *
     * \note Dual variables can only be retrieved for convex continuous models
     * \note Dual values for >= constraints are >= 0
     * \note Dual values for <= constraints are <= 0
     * \note Dual values for == constraints are unconstrained
     *
     * \see https://www.gurobi.com/documentation/9.5/refman/pi.html#attr:Pi
     */
    [[nodiscard]] inline double constraintDualValue(const GRBConstr& constr)
    {
        return std::abs(constr.get(GRB_DoubleAttr_Pi));
    }
    /*!
     * \returns The value of the dual variable for the specific constraint
     *
     * \note Dual variables can only be retrieved for convex continuous models
     * \note Dual values for >= constraints are >= 0
     * \note Dual values for <= constraints are <= 0
     * \note Dual values for == constraints are unconstrained
     *
     * \see https://www.gurobi.com/documentation/9.5/refman/pi.html#attr:Pi
     */
    [[nodiscard]] inline double constraintDualValue(const GRBConstr&& constr)
    {
        return std::abs(constr.get(GRB_DoubleAttr_Pi));
    }
    /*!
     * \returns The value of the dual variable for the specific constraint
     *
     * \note Dual variables can only be retrieved for convex continuous models
     * \note Dual values for >= constraints are >= 0
     * \note Dual values for <= constraints are <= 0
     * \note Dual values for == constraints are unconstrained
     *
     * \see https://www.gurobi.com/documentation/9.5/refman/pi.html#attr:Pi
     */
    [[nodiscard]] inline double constraintDualValue(GRBModel& model, const std::string& constraint_name)
    {
        return constraintDualValue(model.getConstrByName(constraint_name));
    }

    /*!
     * \returns The value of the dual variable for the specific constraint
     *
     * \note Dual variables can only be retrieved for convex continuous models
     * \note Dual values for >= constraints are >= 0
     * \note Dual values for <= constraints are <= 0
     * \note Dual values for == constraints are unconstrained
     *
     * \see https://www.gurobi.com/documentation/9.5/refman/pi.html#attr:Pi
     */
    [[nodiscard]] inline double constraintDualValue(const std::shared_ptr<GRBModel>& model,
                                                    const std::string& constraint_name)
    {
        return constraintDualValue(model->getConstrByName(constraint_name));
    }
}  // namespace grstapse