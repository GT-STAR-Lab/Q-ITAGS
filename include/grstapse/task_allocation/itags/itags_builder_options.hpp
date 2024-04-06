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
#include <cstdint>
#include <set>

namespace grstapse
{
    /*!
     * \brief Container for the options for building an Itags algorithm
     *
     * \see ItagsBuilder
     * \see Itags
     */
    struct ItagsBuilderOptions
    {
        //! The options for the scheduling algorithm
        enum class SchedulerOptions : uint8_t
        {
            e_deterministic_milp = 0,                      //!< \see DeterministicMilpScheduler
            e_monolithic_stochastic_milp,                  //!< \see MonolithicStochasticMilpScheduler
            e_benders_stochastic_milp,                     //!< \see BendersStochasticMilpScheduler
            e_benders_parallel_stochastic_milp,            //!< \see BendersParallelStochasticMilpScheduler
            e_heuristic_approximation_stochastic_milp,     //!< \see HeuristicApproximationStochasticMilpScheduler
            e_gnn_heuristic_approximation_stochastic_milp  //!< \see HeuristicApproximationStochasticMilpScheduler
        };
        SchedulerOptions scheduler = SchedulerOptions::e_deterministic_milp;

        /*!
         * The options for the heuristic
         *
         * \note If you want a heuristic that updates the pruning method then you just need to select that in the
         *       prepruning or postpruning method options (it will automatically be built into the heuristic)
         */
        enum class HeuristicOptions : uint8_t
        {
            //! \see TimeExtendedTaskAllocationQualityBase
            e_tetaq = 0,
            /*!
             * \see NormalizedSchedulerQualityBase
             * \see NsqWithFailureReasonsBase
             */
            e_nsq,
            //! \see AllocationPercentageRemaining
            e_apr
        };
        HeuristicOptions heuristic = HeuristicOptions::e_tetaq;
        float alpha                = 0.5;  //!< Only used when heuristic == e_tetaq

        //! The options for the goal check
        enum class GoalCheckOptions : uint8_t
        {
            //! \see ZeroAprCheck
            e_zero_apr = 0
        };
        GoalCheckOptions goal_check = GoalCheckOptions::e_zero_apr;

        //! The options for the successor generator
        enum class SuccessorGeneratorOptions : uint8_t
        {
            //! \see IncrementalAllocationGenerator
            e_increment = 0
        };
        SuccessorGeneratorOptions successor_generator = SuccessorGeneratorOptions::e_increment;

        //! The options for the memoization of nodes
        enum class MemoizationOptions : uint8_t
        {
            //! \see NullMemoization
            e_null = 0,
            //! \see HashMemoization
            e_hash
        };
        MemoizationOptions memoization = MemoizationOptions::e_hash;

        //! The command line argument options for the prepruning methods
        enum class PrepruningMethodOptions : uint8_t
        {
            //! \see NullPruningMethod
            e_null = 0,
            //! \see TraitsImprovementPruning
            e_no_trait_improvement,
            //! \see ItagsPreviousFailurePruningMethod
            e_previous_failure_reason
        };
        std::set<PrepruningMethodOptions> prepruning = {PrepruningMethodOptions::e_no_trait_improvement};

        //! The command line argument options for the postpruning methods
        enum class PostpruningMethodOptions : uint8_t
        {
            //! \see NullPruningMethod
            e_null = 0
        };
        std::set<PostpruningMethodOptions> postpruning = {PostpruningMethodOptions::e_null};

        bool use_reverse = false;
    };

}  // namespace grstapse