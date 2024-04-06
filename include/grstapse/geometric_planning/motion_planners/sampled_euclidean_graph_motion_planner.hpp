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
// Local
#include "grstapse/geometric_planning/motion_planners/sampled_euclidean_graph_motion_planner_base.hpp"
// endregion

namespace grstapse
{
    //! A motion planner that operates on a SampledEuclideanGraphEnvironment
    class SampledEuclideanGraphMotionPlanner : public SampledEuclideanGraphMotionPlannerBase
    {
       public:
        // region Special Member Functions
        //! No Default Constructor
        SampledEuclideanGraphMotionPlanner() = delete;
        //! No Copy Constructor
        SampledEuclideanGraphMotionPlanner(const SampledEuclideanGraphMotionPlanner&) = delete;
        //! Move Constructor
        SampledEuclideanGraphMotionPlanner(SampledEuclideanGraphMotionPlanner&&) noexcept = default;
        //! Destructor
        virtual ~SampledEuclideanGraphMotionPlanner() = default;
        //! No Copy Assignment Operator
        SampledEuclideanGraphMotionPlanner& operator=(const SampledEuclideanGraphMotionPlanner&) = delete;
        //! Move Assignment Operator
        SampledEuclideanGraphMotionPlanner& operator=(SampledEuclideanGraphMotionPlanner&&) noexcept = default;
        // endregion

        //! Constructor
        SampledEuclideanGraphMotionPlanner(const std::shared_ptr<const ParametersBase>& parameters,
                                           const std::shared_ptr<SampledEuclideanGraphEnvironment>& environment);
    };

}  // namespace grstapse