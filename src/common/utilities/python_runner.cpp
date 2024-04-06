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
#include "grstapse/common/utilities/python_runner.hpp"

namespace grstapse
{
    PythonRunner& PythonRunner::instance()
    {
        static PythonRunner instance;
        return instance;
    }

    pybind11::module& PythonRunner::importModule(const std::string& module_name)
    {
        return instance().importModuleInternal(module_name);
    }

    pybind11::module& PythonRunner::importModuleInternal(const std::string& module_name)
    {
        if(not m_imported_modules.contains(module_name))
        {
            m_imported_modules[module_name] = pybind11::module::import(module_name.c_str());
        }
        return m_imported_modules[module_name];
    }
}  // namespace grstapse