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
#include <map>
// External
#include <pybind11/embed.h>
// endregion

namespace grstapse
{
    /*!
     * \class PythonRunner
     * \brief
     */
    class PythonRunner
    {
       public:
        // region Special Member Functions
       private:
        //! Default Constructor
        PythonRunner() = default;

       public:
        //! Copy Constructor
        PythonRunner(const PythonRunner&) = delete;
        //! Move Constructor
        PythonRunner(PythonRunner&&) noexcept = default;
        //! Destructor
        ~PythonRunner() = default;
        //! Copy Assignment Operator
        PythonRunner& operator=(const PythonRunner&) = delete;
        //! Move Assignment Operator
        PythonRunner& operator=(PythonRunner&&) noexcept = default;
        // endregion

        static pybind11::module& importModule(const std::string& module_name);

       private:
        static PythonRunner& instance();

        pybind11::module& importModuleInternal(const std::string& module_name);

        pybind11::scoped_interpreter m_interpreter;
        std::map<std::string, pybind11::module> m_imported_modules;
    };  // class PythonRunner
}  // namespace grstapse