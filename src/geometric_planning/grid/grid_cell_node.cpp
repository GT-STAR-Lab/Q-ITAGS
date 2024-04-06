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
#include "grstapse/geometric_planning/grid/grid_cell_node.hpp"

// External
#include <boost/functional/hash.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"

namespace grstapse
{
    unsigned int GridCellNode::s_next_id = 0;

    GridCellNode::GridCellNode(unsigned int x, unsigned int y, const std::shared_ptr<const GridCellNode>& parent)
        : GridCell(x, y)
        , AStarSearchNodeBase<GridCellNode>(s_next_id++, parent)
    {}

    unsigned int GridCellNode::hash() const
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, m_x);
        boost::hash_combine(seed, m_y);
        return seed;
    }

    nlohmann::json GridCellNode::serializeToJson(const std::shared_ptr<const ProblemInputs>& problem_inputs) const
    {
        nlohmann::json j;
        j.push_back({{constants::k_x, m_x}, {constants::k_y, m_y}});
        for(std::shared_ptr<const GridCellNode> node = this->parent(); node != nullptr; node = node->parent())
        {
            j.push_back({{constants::k_x, node->m_x}, {constants::k_y, node->m_y}});
        }
        std::reverse(j.begin(), j.end());
        return j;
    }
}  // namespace grstapse