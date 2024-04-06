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
#include <set>
// External
#include <Eigen/Core>

namespace grstapse
{
    // Forward Declarations
    class RobotTraitsMatrixReduction;
    class Task;

    [[nodiscard]] Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                                      const std::vector<unsigned int>& plan_task_indicies);

    [[nodiscard]] Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks);

    [[nodiscard]] Eigen::MatrixXf linearCoefficientMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                                          const std::vector<unsigned int>& plan_task_indicies);

    [[nodiscard]] Eigen::MatrixXf linearCoefficientMatrix(const std::vector<std::shared_ptr<const Task>>& tasks);

    //! \returns The allocated traits matrix
    [[nodiscard]] Eigen::MatrixXf allocatedTraitsMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                        const Eigen::MatrixXf& allocation,
                                                        const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns The traits mismatch matrix
    [[nodiscard]] Eigen::MatrixXf traitsMismatchMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                       const Eigen::MatrixXf& allocation,
                                                       const Eigen::MatrixXf& desired_traits_matrix,
                                                       const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns The traits mismatch matrix with all negative values removed
    [[nodiscard]] Eigen::MatrixXf positiveOnlyTraitsMismatchMatrix(
        const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
        const Eigen::MatrixXf& allocation,
        const Eigen::MatrixXf& desired_traits_matrix,
        const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns The traits mismatch error
    [[nodiscard]] float traitsMismatchError(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                            const Eigen::MatrixXf& allocation,
                                            const Eigen::MatrixXf& desired_traits_matrix,
                                            const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns The traits linear quality
    [[nodiscard]] float traitsLinearQualityCalculator(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                      const Eigen::MatrixXf& allocation,
                                                      const Eigen::MatrixXf& linear_coefficient_matrix,
                                                      const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns A set of the mutex constraints for an allocation
    std::set<std::pair<unsigned int, unsigned int>> computeMutexConstraints(const Eigen::MatrixXf& allocation);

    /*!
     * Updates the set by adding transitive constraints (0,1) ^ (1,2) -> (0, 2)
     *
     * \returns The updated set
     */
    std::set<std::pair<unsigned int, unsigned int>> addPrecedenceTransitiveConstraints(
        std::set<std::pair<unsigned int, unsigned int>> ordering_constraints);
}  // namespace grstapse