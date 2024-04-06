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
#include "grstapse/task_allocation/itags/normalized_allocation_quality.hpp"

// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

namespace grstapse
{
    NormalizedAllocationQuality::NormalizedAllocationQuality(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {
        setMaxAndMinQuality(problem_inputs);
    }

    float NormalizedAllocationQuality::operator()(const std::shared_ptr<IncrementalTaskAllocationNode>& node) const
    {
        const Eigen::MatrixXf& allocation = node->allocation();
        const float traits_linear_quality =
            traitsLinearQualityCalculator(*m_problem_inputs->robotTraitsMatrixReduction(),
                                          allocation,
                                          m_problem_inputs->linearCoefficientMatrix(),
                                          m_problem_inputs->teamTraitsMatrix());

        // ||max(E(A), 0)||_{1, 1} / ||Y||_{1,1}
        return (m_max_quality - traits_linear_quality) / (m_max_quality - m_min_quality);
    }

    void NormalizedAllocationQuality::setMaxAndMinQuality(
        const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
    {
        const Eigen::MatrixXf& allocation_max = Eigen::MatrixXf::Ones(problem_inputs->linearCoefficientMatrix().rows(),
                                                                      problem_inputs->linearCoefficientMatrix().cols());
        m_max_quality = traitsLinearQualityCalculator(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                      allocation_max,
                                                      m_problem_inputs->linearCoefficientMatrix(),
                                                      m_problem_inputs->teamTraitsMatrix());

        const Eigen::MatrixXf& allocation_min = Eigen::MatrixXf::Zero(problem_inputs->linearCoefficientMatrix().rows(),
                                                                      problem_inputs->linearCoefficientMatrix().cols());
        m_min_quality = traitsLinearQualityCalculator(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                      allocation_min,
                                                      m_problem_inputs->linearCoefficientMatrix(),
                                                      m_problem_inputs->teamTraitsMatrix());
    }
}  // namespace grstapse