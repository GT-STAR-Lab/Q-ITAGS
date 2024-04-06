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
#include <optional>
// External
#include <Eigen/Core>
// Local
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search_node_base.hpp"
#include "grstapse/common/utilities/matrix_dimensions.hpp"
#include "grstapse/task_allocation/assignment.hpp"

namespace grstapse
{
    // Forward Declaration
    class ScheduleBase;

    //! \brief A node that contains an allocation of agents to tasks
    class IncrementalTaskAllocationNode : public GreedyBestFirstSearchNodeBase<IncrementalTaskAllocationNode>
    {
        using Base_ = GreedyBestFirstSearchNodeBase<IncrementalTaskAllocationNode>;

       public:
        /*!
         * \brief Constructor for the root node
         * \param dimensions The dimensions of the allocation matrix this node represents
         */
        explicit IncrementalTaskAllocationNode(const MatrixDimensions& dimensions, bool use_reverse = false);

        /*!
         * \brief Constructor for any node except the root node
         *
         * \param assignment The last assignment for the allocation matrix this node represents
         * \param parent The parent of this node
         */
        IncrementalTaskAllocationNode(const Assignment& assignment,
                                      const std::shared_ptr<const IncrementalTaskAllocationNode>& parent,
                                      bool use_reverse = false);

        //! \returns The last assigment (robot, task)
        [[nodiscard]] inline const std::optional<Assignment>& lastAssigment() const;

        //! \returns The dimensions of the allocation matrix (MxN)
        [[nodiscard]] const MatrixDimensions& matrixDimensions() const;

        /*!
         * \returns The allocation contained by this node
         *
         * \note Virtual for unit tests
         */
        virtual Eigen::MatrixXf allocation() const;

        //! Sets the schedule for this node
        inline void setSchedule(const std::shared_ptr<const ScheduleBase>& schedule);

        //! \returns The schedule computed for this node if NSQ was run
        [[nodiscard]] inline const std::shared_ptr<const ScheduleBase>& schedule() const;

        //! \returns A hash for this node
        unsigned int hash() const final override;

        //! \copydoc SearchNodeBase
        [[nodiscard]] nlohmann::json serializeToJson(
            const std::shared_ptr<const ProblemInputs>& problem_inputs) const override;

       private:
        std::optional<Assignment> m_last_assigment;
        std::optional<MatrixDimensions> m_matrix_dimensions;
        std::shared_ptr<const ScheduleBase> m_schedule;
        bool m_use_reverse;

        static unsigned int s_next_id;
    };

    // Inline functions
    const std::optional<Assignment>& IncrementalTaskAllocationNode::lastAssigment() const
    {
        return m_last_assigment;
    }

    void IncrementalTaskAllocationNode::setSchedule(const std::shared_ptr<const ScheduleBase>& schedule)
    {
        m_schedule = schedule;
    }

    const std::shared_ptr<const ScheduleBase>& IncrementalTaskAllocationNode::schedule() const
    {
        return m_schedule;
    }
}  // namespace grstapse