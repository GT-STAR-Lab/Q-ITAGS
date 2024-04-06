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
#include "grstapse/parameters/algorithm_parameters_factory_base.hpp"
// endregion

namespace grstapse
{
    /*!
     * \class MotionPlannerParametersFactory
     * \brief
     */
    class MotionPlannerParametersFactory : public AlgorithmParametersFactoryBase
    {
       public:
        static MotionPlannerParametersFactory& instance();
        // region Special Member Functions
       private:
        //! Default Constructor
        MotionPlannerParametersFactory();

       public:
        //! Copy Constructor
        MotionPlannerParametersFactory(const MotionPlannerParametersFactory&) = delete;
        //! Move Constructor
        MotionPlannerParametersFactory(MotionPlannerParametersFactory&&) noexcept = delete;
        //! Destructor
        ~MotionPlannerParametersFactory() = default;
        //! Copy Assignment Operator
        MotionPlannerParametersFactory& operator=(const MotionPlannerParametersFactory&) = delete;
        //! Move Assignment Operator
        MotionPlannerParametersFactory& operator=(MotionPlannerParametersFactory&&) noexcept = delete;
        // endregion
    };  // class MotionPlannerParametersFactory
}  // namespace grstapse