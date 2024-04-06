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
#include <gtest/gtest.h>
// Project
#include <grstapse/task_allocation/itags/task_allocation_math.hpp>

namespace grstapse::unittests
{
    TEST(TaskAllocationMath, addPrecedenceTransitiveConstraints)
    {
        {
            std::set<std::pair<unsigned int, unsigned int>> inputs = {{0, 1}, {1, 2}};
            auto output                                            = addPrecedenceTransitiveConstraints(inputs);
            ASSERT_EQ(output.size(), 3);
            ASSERT_TRUE(output.contains({0, 2}));
        }

        {
            std::set<std::pair<unsigned int, unsigned int>> inputs = {{0, 1}, {2, 1}};
            auto output                                            = addPrecedenceTransitiveConstraints(inputs);
            ASSERT_EQ(output.size(), 2);
        }

        {
            std::set<std::pair<unsigned int, unsigned int>> inputs = {{0, 1}, {1, 2}, {2, 3}};
            auto output                                            = addPrecedenceTransitiveConstraints(inputs);
            ASSERT_EQ(output.size(), 6);
        }
    }

}  // namespace grstapse::unittests