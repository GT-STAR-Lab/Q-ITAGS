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
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"

// Local
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"

namespace grstapse
{
    Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                        const std::vector<unsigned int>& plan_task_indicies)
    {
        if(tasks.empty() || plan_task_indicies.empty())
        {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = tasks.front()->desiredTraits().size();

        Eigen::MatrixXf rv(plan_task_indicies.size(), num_traits);
        unsigned int row_nr = 0;
        for(const unsigned int index: plan_task_indicies)
        {
            rv.row(row_nr++) = tasks[index]->desiredTraits();
        }
        return rv;
    }

    Eigen::MatrixXf linearCoefficientMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                            const std::vector<unsigned int>& plan_task_indicies)
    {
        if(tasks.empty() || plan_task_indicies.empty())
        {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = tasks.front()->linearCoefficients().size();

        Eigen::MatrixXf rv(plan_task_indicies.size(), num_traits);
        unsigned int row_nr = 0;
        for(const unsigned int index: plan_task_indicies)
        {
            rv.row(row_nr++) = tasks[index]->linearCoefficients();
        }
        return rv;
    }

    Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks)
    {
        if(tasks.empty())
        {
            return Eigen::MatrixXf();
        }
        const unsigned int num_traits = tasks.front()->desiredTraits().size();
        Eigen::MatrixXf rv(tasks.size(), num_traits);
        unsigned int row_nr = 0;
        for(const std::shared_ptr<const Task>& task: tasks)
        {
            rv.row(row_nr++) = task->desiredTraits();
        }
        return rv;
    }

    Eigen::MatrixXf linearCoefficientMatrix(const std::vector<std::shared_ptr<const Task>>& tasks)
    {
        if(tasks.empty())
        {
            return Eigen::MatrixXf();
        }
        const unsigned int num_traits = tasks.front()->linearCoefficients().size();
        Eigen::MatrixXf rv(tasks.size(), num_traits);
        unsigned int row_nr = 0;
        for(const std::shared_ptr<const Task>& task: tasks)
        {
            rv.row(row_nr++) = task->linearCoefficients();
        }
        return rv;
    }

    Eigen::MatrixXf allocatedTraitsMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                          const Eigen::MatrixXf& allocation,
                                          const Eigen::MatrixXf& robot_traits_matrix)
    {
        return robot_traits_matrix_reduction.reduce(allocation, robot_traits_matrix);
    }

    Eigen::MatrixXf traitsMismatchMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                         const Eigen::MatrixXf& allocation,
                                         const Eigen::MatrixXf& desired_traits_matrix,
                                         const Eigen::MatrixXf& robot_traits_matrix)
    {
        // A * Q
        Eigen::MatrixXf allocated_traits_matrix =
            allocatedTraitsMatrix(robot_traits_matrix_reduction, allocation, robot_traits_matrix);

        // E(A) = Y - A * Q
        return desired_traits_matrix - allocated_traits_matrix;
    }

    Eigen::MatrixXf positiveOnlyTraitsMismatchMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                     const Eigen::MatrixXf& allocation,
                                                     const Eigen::MatrixXf& desired_traits_matrix,
                                                     const Eigen::MatrixXf& robot_traits_matrix)
    {
        Eigen::MatrixXf traits_mismatch_matrix =
            traitsMismatchMatrix(robot_traits_matrix_reduction, allocation, desired_traits_matrix, robot_traits_matrix);

        return (traits_mismatch_matrix.array() < 0).select(0, traits_mismatch_matrix);
    }

    float traitsMismatchError(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                              const Eigen::MatrixXf& allocation,
                              const Eigen::MatrixXf& desired_traits_matrix,
                              const Eigen::MatrixXf& robot_traits_matrix)
    {
        return positiveOnlyTraitsMismatchMatrix(robot_traits_matrix_reduction,
                                                allocation,
                                                desired_traits_matrix,
                                                robot_traits_matrix)
            .sum();
    }

    float traitsLinearQualityCalculator(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                        const Eigen::MatrixXf& allocation,
                                        const Eigen::MatrixXf& linear_coefficient_matrix,
                                        const Eigen::MatrixXf& robot_traits_matrix)
    {
        // A * Q
        Eigen::MatrixXf allocated_traits_matrix =
            allocatedTraitsMatrix(robot_traits_matrix_reduction, allocation, robot_traits_matrix);

        return allocated_traits_matrix.cwiseProduct(linear_coefficient_matrix).sum();
    }

    std::set<std::pair<unsigned int, unsigned int>> computeMutexConstraints(const Eigen::MatrixXf& allocation)
    {
        if(allocation.isZero())
        {
            return std::set<std::pair<unsigned int, unsigned int>>();
        }

        std::set<std::pair<unsigned int, unsigned int>> mutex_constraints;
        const unsigned int num_tasks  = allocation.rows();
        const unsigned int num_robots = allocation.cols();
        for(unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
        {
            std::vector<unsigned int> allocated_tasks;
            for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
            {
                if(allocation(task_nr, robot_nr))
                {
                    allocated_tasks.push_back(task_nr);
                }
            }
            if(!allocated_tasks.empty())
            {
                for(unsigned int task_i_index = 0, num_assigned_tasks = allocated_tasks.size();
                    task_i_index < num_assigned_tasks;
                    ++task_i_index)
                {
                    const unsigned int task_i = allocated_tasks[task_i_index];
                    for(unsigned int task_j_index = task_i_index + 1; task_j_index < num_assigned_tasks; ++task_j_index)
                    {
                        const unsigned int task_j = allocated_tasks[task_j_index];
                        mutex_constraints.insert({task_i, task_j});
                    }
                }
            }
        }
        return mutex_constraints;
    }

    std::set<std::pair<unsigned int, unsigned int>> addPrecedenceTransitiveConstraints(
        std::set<std::pair<unsigned int, unsigned int>> ordering_constraints)
    {
        unsigned int num_ordering_constraints = ordering_constraints.size();
        while(true)
        {
            std::set<std::pair<unsigned int, unsigned int>> temp;
            for(const std::pair<unsigned int, unsigned int>& constraint1: ordering_constraints)
            {
                temp.insert(constraint1);
                for(const std::pair<unsigned int, unsigned int>& constraint2: ordering_constraints)
                {
                    if(constraint1.second == constraint2.first)
                    {
                        temp.insert(std::pair(constraint1.first, constraint2.second));
                    }
                }
            }
            ordering_constraints = std::move(temp);
            if(num_ordering_constraints == ordering_constraints.size())
            {
                break;
            }
            num_ordering_constraints = ordering_constraints.size();
        }
        return ordering_constraints;
    }
}  // namespace grstapse