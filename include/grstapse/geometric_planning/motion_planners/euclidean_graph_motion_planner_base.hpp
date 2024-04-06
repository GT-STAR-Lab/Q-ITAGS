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
//  Local
#include "grstapse/geometric_planning/motion_planners/graph_motion_planner_base.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    class EuclideanGraphMotionPlannerParameters;
    class EuclideanGraphEnvironmentBase;
    // endregion

    //! A base class for motion planners that operate on am euclidean graph
    class EuclideanGraphMotionPlannerBase : public GraphMotionPlannerBase
    {
       public:
        // region Special Member Functions
        //! No Default Constructor
        EuclideanGraphMotionPlannerBase() = delete;
        //! No Copy Constructor
        EuclideanGraphMotionPlannerBase(const EuclideanGraphMotionPlannerBase&) = delete;
        //! Move Constructor
        EuclideanGraphMotionPlannerBase(EuclideanGraphMotionPlannerBase&&) noexcept = default;
        //! Destructor
        virtual ~EuclideanGraphMotionPlannerBase() = default;
        //! No Copy Assignment Operator
        EuclideanGraphMotionPlannerBase& operator=(const EuclideanGraphMotionPlannerBase&) = delete;
        //! Move Assignment Operator
        EuclideanGraphMotionPlannerBase& operator=(EuclideanGraphMotionPlannerBase&&) noexcept = default;
        // endregion

       protected:
        //! Constructor
        EuclideanGraphMotionPlannerBase(const std::shared_ptr<const ParametersBase>& parameters,
                                        const std::shared_ptr<EuclideanGraphEnvironmentBase>& environment);
    };

}  // namespace grstapse