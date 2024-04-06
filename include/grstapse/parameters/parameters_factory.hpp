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
#include <memory>
// External
#include <nlohmann/json.hpp>
// endregion

namespace grstapse
{
    // region Forward Declarations
    class SchedulerParametersFactory;
    class SearchParametersFactory;
    class MotionPlannerParametersFactory;
    class ParametersBase;
    // endregion

    /*!
     * \class ParametersFactory
     * \brief A factory for loading parameters for various algorithms
     */
    class ParametersFactory
    {
       public:
        /*!
         * The type of parameter to create
         */
        enum class Type : uint8_t
        {
            e_motion_planner,
            e_scheduler,
            e_search
        };

        //! \returns The singleton
        static ParametersFactory& instance();

        //! Creates a parameters container based on the \p type and the loaded json
        std::shared_ptr<ParametersBase> create(Type type, const nlohmann::json& j) const;

        // region Special Member Functions
       private:
        //! Default Constructor
        ParametersFactory();

       public:
        //! Copy Constructor
        ParametersFactory(const ParametersFactory&) = delete;
        //! Move Constructor
        ParametersFactory(ParametersFactory&&) = delete;
        //! Destructor
        ~ParametersFactory() = default;
        //! Copy Assignment Operator
        ParametersFactory& operator=(const ParametersFactory&) = delete;
        //! Move Assignment Operator
        ParametersFactory& operator=(ParametersFactory&&) = delete;

        MotionPlannerParametersFactory& m_motion_planner_parameters_factory;
        SchedulerParametersFactory& m_scheduler_parameters_factory;
        SearchParametersFactory& m_search_parameters_factory;
        //  endregion
    };  // class ParametersFactory
}  // namespace grstapse