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
// Project
#include <grstapse/common/utilities/json_extension.hpp>
#include <grstapse/config.hpp>
#include <grstapse/geometric_planning/environments/pgm_ompl_environment.hpp>
#include <grstapse/geometric_planning/motion_planning_enums.hpp>

namespace grstapse::unittests
{
    TEST(PgmEnvironment, LoadEmpty)
    {
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_empty.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }

    TEST(PgmEnvironment, LoadDublin)
    {
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_dubins.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }

    TEST(PgmEnvironment, LoadBlobs)
    {
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_blobs.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }

    TEST(PgmEnvironment, LoadCenterBlock)
    {
        std::ifstream fin(std::string(s_data_dir) +
                          std::string("/geometric_planning/environments/pgm_center_block.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }

    TEST(PgmEnvironment, LoadSegmented)
    {
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_segmented.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }

    TEST(PgmEnvironment, LoadWall)
    {
        std::ifstream fin(std::string(s_data_dir) + std::string("/geometric_planning/environments/pgm_wall.json"));
        nlohmann::json j;
        fin >> j;
        auto environment = j.get<std::shared_ptr<PgmOmplEnvironment>>();

        ASSERT_FLOAT_EQ(environment->resolution(), 0.05);
        ASSERT_FLOAT_EQ(environment->minX(), -51.224998);
        // ASSERT_NEAR(environment->maxX(), 51.224998, 1e-3);
        ASSERT_FLOAT_EQ(environment->minY(), -51.224998);
        // ASSERT_NEAR(environment->maxY(), 51.224998, 1e-3);
        ASSERT_EQ(environment->stateSpaceType(), OmplStateSpaceType::e_se2);
    }
}  // namespace grstapse::unittests