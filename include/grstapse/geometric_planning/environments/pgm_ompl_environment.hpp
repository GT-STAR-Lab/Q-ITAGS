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

// Global
#include <memory>
// Local
#include "grstapse/common/utilities/pgm.hpp"
#include "grstapse/geometric_planning/environments/ompl_environment.hpp"

namespace grstapse
{
    //! Environment where the map comes from a PGM image
    class PgmOmplEnvironment : public OmplEnvironment
    {
       public:
        //! For json
        PgmOmplEnvironment();

        //! Constructor
        PgmOmplEnvironment(const std::string& filepath,
                           const float resolution,
                           const float origin_x,
                           const float origin_y);

        //! \copydoc ompl::base::StateValidityChecker
        [[nodiscard]] bool isValid(const ompl::base::State* state) const final override;

        //! \copydoc Environment
        [[nodiscard]] float longestPath() const final override;

        //! \returns The minimum x coordinate in the environment
        [[nodiscard]] inline float minX() const;

        //! \returns The maximum x coordinate in the environment
        [[nodiscard]] inline float maxX() const;

        //! \returns The minimum y coordinate in the environment
        [[nodiscard]] inline float minY() const;

        //! \returns The maximum x coordinate in the environment
        [[nodiscard]] inline float maxY() const;

        //! \returns The resolution of a pixel in the pgm
        [[nodiscard]] inline float resolution() const;

       private:
        //! \returns The cell coordinate in the image for the real word coordinates (\p x, \p y)
        [[nodiscard]] inline std::pair<int, int> toCell(const float x, const float y) const;

        Pgm m_pgm;
        float m_turning_radius;
        float m_resolution;
        float m_origin_x;
        float m_origin_y;

        friend void from_json(const nlohmann::json& j, PgmOmplEnvironment& e);
    };

    void from_json(const nlohmann::json& j, PgmOmplEnvironment& e);

    // Inline functions
    float PgmOmplEnvironment::minX() const
    {
        return m_origin_x;
    }

    float PgmOmplEnvironment::maxX() const
    {
        return m_origin_x + m_pgm.width() * m_resolution;
    }

    float PgmOmplEnvironment::minY() const
    {
        return m_origin_y;
    }

    float PgmOmplEnvironment::maxY() const
    {
        return m_origin_y + m_pgm.height() * m_resolution;
    }

    float PgmOmplEnvironment::resolution() const
    {
        return m_resolution;
    }

    std::pair<int, int> PgmOmplEnvironment::toCell(const float x, const float y) const
    {
        const int cx = (x - m_origin_x) / m_resolution;
        const int cy = (y - m_origin_y) / m_resolution;
        return {cx, cy};
    }
}  // namespace grstapse