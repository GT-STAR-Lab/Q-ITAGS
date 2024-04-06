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
// External
#include <cli11/cli11.hpp>
#include <fmt/format.h>
// Project
#include <grstapse/core.hpp>

namespace grstapse::main
{
    void createItagsSubcommand() {}

    int main(int argc, char** argv)
    {
        CLI::App app{"Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling"};
        app.formatter(std::make_shared<cli11_ext::CustomCliFormatter>());

        try
        {
            app.parse(argc, argv);
        }
        catch(const CLI::ParseError& e)
        {
            return app.exit(e);
        }

        return 0;
    }
}  // namespace grstapse::main

int main(int argc, char** argv)
{
    return grstapse::main::main(argc, argv);
}