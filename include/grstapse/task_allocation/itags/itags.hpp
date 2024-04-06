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
#include <fstream>
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search.hpp"
#include "grstapse/common/search/hash_memoization.hpp"
#include "grstapse/common/utilities/matrix_dimensions.hpp"
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/incremental_allocation_generator.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/itags_statistics.hpp"
#include "grstapse/task_allocation/itags/normalized_schedule_quality.hpp"
#include "grstapse/task_allocation/itags/time_extended_task_allocation_metric.hpp"
#include "grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp"
#include "grstapse/task_allocation/itags/traits_improvement_pruning.hpp"
#include "grstapse/task_allocation/itags/zero_apr_check.hpp"
#include "grstapse/task_allocation/itags/zero_pos_check.hpp"

namespace grstapse
{
    namespace detail
    {
        /*!
         * \brief Hack for designated initializers
         */
        struct ItagsParametersImpl
        {
            std::shared_ptr<const ItagsProblemInputs> problem_inputs;
            std::shared_ptr<const HeuristicBase<IncrementalTaskAllocationNode>> heuristic                    = nullptr;
            std::shared_ptr<const SuccessorGeneratorBase<IncrementalTaskAllocationNode>> successor_generator = nullptr;
            std::shared_ptr<const GoalCheckBase<IncrementalTaskAllocationNode>> goal_check                   = nullptr;
            std::shared_ptr<const MemoizationBase<IncrementalTaskAllocationNode>> memoization                = nullptr;
            std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>> pre_pruning_method             = nullptr;
            std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>> post_pruning_method            = nullptr;
            bool use_reverse                                                                                 = false;
        };
    }  // namespace detail

    /*!
     * \brief The Incremental Task Allocation Graph Search
     *
     * A heuristic search used for trait-based time extended task allocation problems
     *
     * \tparam Heur The heuristic to be used during search
     *
     * \cite Neville, G., Messing, A., Ravichandar, H., Hutchinson, S., & Chernova, S. (2021, August). An interleaved
     *       approach to trait-based task allocation and scheduling. In 2021 IEEE/RSJ International Conference on
     *       Intelligent Robots and Systems (IROS) (pp. 1507-1514). IEEE.
     */
    class Itags : public GreedyBestFirstSearch<IncrementalTaskAllocationNode, ItagsStatistics>
    {
        using Base_ = GreedyBestFirstSearch<IncrementalTaskAllocationNode, ItagsStatistics>;

       public:
        /*!
         * \brief Constructor
         *
         * \param problem_inputs
         * \param heuristic
         * \param successor_generator
         * \param goal_check
         * \param memoization
         * \param pre_pruning_method
         * \param post_pruning_method
         */
        explicit Itags(
            const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
            const std::shared_ptr<const HeuristicBase<IncrementalTaskAllocationNode>>& heuristic,
            const std::shared_ptr<const SuccessorGeneratorBase<IncrementalTaskAllocationNode>>& successor_generator,
            const std::shared_ptr<const GoalCheckBase<IncrementalTaskAllocationNode>>& goal_check,
            const std::shared_ptr<const MemoizationBase<IncrementalTaskAllocationNode>>& memoization,
            const std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>>& pre_pruning_method,
            const std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>>& post_pruning_method,
            bool use_reverse);

        /*!
         * \brief Factory function
         *
         * This can be considered the factory function for the default ITAGS
         *
         * \param problem_inputs Inputs from the problem
         */
        explicit Itags(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
            : Itags(problem_inputs,
                    std::make_shared<const TimeExtendedTaskAllocationQuality>(problem_inputs),
                    std::make_shared<const IncrementalAllocationGenerator>(problem_inputs),
                    std::make_shared<const ZeroAprCheck>(problem_inputs),
                    std::make_shared<const HashMemoization<IncrementalTaskAllocationNode>>(),
                    std::make_shared<TraitsImprovementPruning>(problem_inputs),
                    std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>(),
                    problem_inputs->useReverse())
        {}

        /*!
         * \brief Factory function
         *
         * This can be considered the factory function for the reverseITAGS
         *
         * \param problem_inputs Inputs from the problem
         */
        explicit Itags(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs, bool forward_Search)
            : Itags(problem_inputs,
                    nullptr,
                    std::make_shared<const IncrementalAllocationGenerator>(problem_inputs),
                    nullptr,
                    std::make_shared<const HashMemoization<IncrementalTaskAllocationNode>>(),
                    nullptr,
                    std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>(),
                    problem_inputs->useReverse())
        {
            if(forward_Search)
            {
                m_heuristic         = std::make_shared<const TimeExtendedTaskAllocationQuality>(problem_inputs);
                m_goal_check        = std::make_shared<const ZeroAprCheck>(problem_inputs);
                m_prepruning_method = std::make_shared<TraitsImprovementPruning>(problem_inputs);
            }
            else
            {
                m_heuristic         = std::make_shared<const TimeExtendedTaskAllocationMetric>(problem_inputs);
                m_goal_check        = std::make_shared<const ZeroPosCheck>(problem_inputs);
                m_prepruning_method = std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>();
            }
        }

        /*!
         * \brief Factory function
         *
         * \param parameters Designated parameters handler for customizing the search
         */
        explicit Itags(const detail::ItagsParametersImpl& parameters)
            : Base_{parameters.problem_inputs->itagsParameters(),
                    {.heuristic =
                         parameters.heuristic != nullptr
                             ? parameters.heuristic
                             : std::make_shared<const TimeExtendedTaskAllocationQuality>(parameters.problem_inputs),
                     .successor_generator =
                         parameters.successor_generator != nullptr
                             ? parameters.successor_generator
                             : std::make_shared<const IncrementalAllocationGenerator>(parameters.problem_inputs),
                     .goal_check         = parameters.goal_check != nullptr
                                               ? parameters.goal_check
                                               : std::make_shared<const ZeroAprCheck>(parameters.problem_inputs),
                     .memoization        = parameters.memoization != nullptr
                                               ? parameters.memoization
                                               : std::make_shared<const HashMemoization<IncrementalTaskAllocationNode>>(),
                     .prepruning_method  = parameters.pre_pruning_method != nullptr
                                               ? parameters.pre_pruning_method
                                               : std::make_shared<TraitsImprovementPruning>(parameters.problem_inputs),
                     .postpruning_method = parameters.post_pruning_method != nullptr
                                               ? parameters.post_pruning_method
                                               : std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>()}}
            , m_problem_inputs(parameters.problem_inputs)
            , m_use_reverse(parameters.use_reverse)
        {}

        /*!
         * \brief Factory function
         *
         * \param parameters Designated parameters handler for customizing the search
         */
        explicit Itags(detail::ItagsParametersImpl&& parameters)
            : Base_{parameters.problem_inputs->itagsParameters(),
                    {.heuristic =
                         parameters.heuristic != nullptr
                             ? std::move(parameters.heuristic)
                             : std::make_shared<const TimeExtendedTaskAllocationQuality>(parameters.problem_inputs),
                     .successor_generator =
                         parameters.successor_generator != nullptr
                             ? std::move(parameters.successor_generator)
                             : std::make_shared<const IncrementalAllocationGenerator>(parameters.problem_inputs),
                     .goal_check         = parameters.goal_check != nullptr
                                               ? std::move(parameters.goal_check)
                                               : std::make_shared<const ZeroAprCheck>(parameters.problem_inputs),
                     .memoization        = parameters.memoization != nullptr
                                               ? std::move(parameters.memoization)
                                               : std::make_shared<const HashMemoization<IncrementalTaskAllocationNode>>(),
                     .prepruning_method  = parameters.pre_pruning_method != nullptr
                                               ? std::move(parameters.pre_pruning_method)
                                               : std::make_shared<TraitsImprovementPruning>(parameters.problem_inputs),
                     .postpruning_method = parameters.post_pruning_method != nullptr
                                               ? std::move(parameters.post_pruning_method)
                                               : std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>()}}
            , m_problem_inputs(parameters.problem_inputs)
            , m_use_reverse(parameters.use_reverse)
        {}

        /*!
         * \returns Whether the specified problem can be allocated
         */
        [[nodiscard]] bool isAllocatable() const;

        /*!
         * \copydoc BestFirstSearchBase
         */
        [[nodiscard]] std::shared_ptr<IncrementalTaskAllocationNode> createRootNode() override;

       private:
        std::shared_ptr<const ItagsProblemInputs> m_problem_inputs;
        bool m_use_reverse;
    };
}  // namespace grstapse