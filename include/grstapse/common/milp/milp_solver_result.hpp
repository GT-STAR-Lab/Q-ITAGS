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

// External Forward Declarations
class GRBModel;

namespace grstapse
{
    // Forward Declarations
    class FailureReason;

    //! The result of solving a milp optimization (contains the resulting model or the reason for failure)
    class MilpSolverResult
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param model The resulting model from the optimization
         */
        explicit MilpSolverResult(const std::shared_ptr<GRBModel>& model);

        /*!
         * \brief Constructor
         *
         * \param failure_reason The reason that the MILP construction or optimization failed
         */
        explicit MilpSolverResult(const std::shared_ptr<const FailureReason>& failure_reason,
                                  unsigned int num_iterations = 0);

        //! \returns Whether the MILP optimization succeeded
        [[nodiscard]] inline bool success() const;

        //! \returns The model that a solver attempted to solve
        [[nodiscard]] inline const std::shared_ptr<GRBModel>& model() const;

        //! Increments the number of iterations used to solve the problem
        inline void incrementNumIterations();

        //! \returns The number of iterations needed to solve the problem
        [[nodiscard]] inline unsigned int numIterations() const;

        //! \returns Whether the MILP optimization failed
        [[nodiscard]] inline bool failure() const;

        //! \returns The reason the MILP optimization failed
        [[nodiscard]] inline const std::shared_ptr<const FailureReason>& failureReason() const;

       private:
        std::shared_ptr<GRBModel> m_model;
        std::shared_ptr<const FailureReason> m_failure_reason;
        unsigned int m_num_iterations;
    };

    // Inline Functions
    bool MilpSolverResult::success() const
    {
        return m_model != nullptr;
    }

    const std::shared_ptr<GRBModel>& MilpSolverResult::model() const
    {
        return m_model;
    }

    void MilpSolverResult::incrementNumIterations()
    {
        ++m_num_iterations;
    }

    unsigned int MilpSolverResult::numIterations() const
    {
        return m_num_iterations;
    }

    bool MilpSolverResult::failure() const
    {
        return m_failure_reason != nullptr;
    }

    const std::shared_ptr<const FailureReason>& MilpSolverResult::failureReason() const
    {
        return m_failure_reason;
    }
}  // namespace grstapse