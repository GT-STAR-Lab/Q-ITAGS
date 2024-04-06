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

// Global
#include <fstream>
// External
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
// Project
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/environments/euclidean_graph_environment.hpp>
#include <grstapse/geometric_planning/environments/sampled_euclidean_graph_environment.hpp>

namespace grstapse::unittests
{
    TEST(SampledEuclideanGraphEnvironment, Load)
    {
        std::ifstream in(std::string(s_data_dir) +
                         std::string("/geometric_planning/environments/sampled_euclidean_graph.json"));
        nlohmann::json j;
        in >> j;
        auto environment = j.get<std::shared_ptr<SampledEuclideanGraphEnvironment>>();
        ASSERT_EQ(environment->numGraphs(), 3);
        ASSERT_EQ(environment->graph(0)->numVertices(), 19);
        ASSERT_EQ(environment->graph(0)->numEdges(), 22);
        ASSERT_EQ(environment->graph(1)->numVertices(), 19);
        ASSERT_EQ(environment->graph(1)->numEdges(), 8);
        ASSERT_EQ(environment->graph(2)->numVertices(), 19);
        ASSERT_EQ(environment->graph(2)->numEdges(), 10);
    }
}  // namespace grstapse::unittests