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
#include "grstapse/task_allocation/itags/itags_builder.hpp"

// Local
#include "grstapse/common/search/disjunctive_pruning_method.hpp"
#include "grstapse/scheduling/milp/stochastic/benders/benders_parallel_stochastic_milp_scheduler.hpp"
#include "grstapse/scheduling/milp/stochastic/benders/benders_stochastic_milp_scheduler.hpp"
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/gnn_scenario_selector.hpp"
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/heuristic_approximation_stochastic_scheduler.hpp"
#include "grstapse/scheduling/milp/stochastic/monolithic/monolithic_stochastic_milp_scheduler.hpp"
#include "grstapse/task_allocation/itags/itags.hpp"
#include "grstapse/task_allocation/itags/itags_builder_options.hpp"
#include "grstapse/task_allocation/itags/itags_previous_failure_pruning_method.hpp"

namespace grstapse
{
    ItagsBuilder::ItagsBuilder(const ItagsBuilderOptions& builder_options)
        : m_builder_options(builder_options)
    {}

    std::shared_ptr<Itags> ItagsBuilder::build(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs) const
    {
        std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>> prepruning;
        // region prepruning
        std::shared_ptr<ItagsPreviousFailurePruningMethod> previous_failure_pruning_method = nullptr;
        if(m_builder_options.prepruning.empty())
        {
            prepruning = std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>();
        }
        else if(m_builder_options.prepruning.size() == 1)
        {
            switch(*(m_builder_options.prepruning.begin()))
            {
                case ItagsBuilderOptions::PrepruningMethodOptions::e_null:
                {
                    prepruning = std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>();
                    break;
                }
                case ItagsBuilderOptions::PrepruningMethodOptions::e_no_trait_improvement:
                {
                    prepruning = std::make_shared<TraitsImprovementPruning>(problem_inputs);
                    break;
                }
                case ItagsBuilderOptions::PrepruningMethodOptions::e_previous_failure_reason:
                {
                    prepruning = previous_failure_pruning_method =
                        std::make_shared<ItagsPreviousFailurePruningMethod>(problem_inputs);
                    break;
                }
                default:
                {
                    throw createLogicError("Unknown prepruning method");
                }
            }
        }
        else
        {
            auto tmp = std::make_shared<DisjunctivePruningMethod<IncrementalTaskAllocationNode>>();
            for(ItagsBuilderOptions::PrepruningMethodOptions elem: m_builder_options.prepruning)
            {
                switch(elem)
                {
                    case ItagsBuilderOptions::PrepruningMethodOptions::e_null:
                    {
                        // Ignore
                        break;
                    }
                    case ItagsBuilderOptions::PrepruningMethodOptions::e_no_trait_improvement:
                    {
                        tmp->add(std::make_shared<TraitsImprovementPruning>(problem_inputs));
                        break;
                    }
                    case ItagsBuilderOptions::PrepruningMethodOptions::e_previous_failure_reason:
                    {
                        tmp->add(previous_failure_pruning_method =
                                     std::make_shared<ItagsPreviousFailurePruningMethod>(problem_inputs));
                        break;
                    }
                    default:
                    {
                        throw createLogicError("Unknown prepruning method");
                    }
                }
            }
            prepruning = std::move(tmp);
        }
        // endregion

        std::shared_ptr<PruningMethodBase<IncrementalTaskAllocationNode>> postpruning;
        // region postpruning
        if(m_builder_options.postpruning.empty())
        {
            postpruning = std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>();
        }
        else if(m_builder_options.postpruning.size() == 1)
        {
            switch(*(m_builder_options.postpruning.begin()))
            {
                case ItagsBuilderOptions::PostpruningMethodOptions::e_null:
                {
                    postpruning = std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>();
                    break;
                }
                default:
                {
                    throw createLogicError("Unknown postpruning method");
                }
            }
        }
        else
        {
            auto tmp = std::make_shared<DisjunctivePruningMethod<IncrementalTaskAllocationNode>>();
            for(ItagsBuilderOptions::PostpruningMethodOptions elem: m_builder_options.postpruning)
            {
                switch(elem)
                {
                    case ItagsBuilderOptions::PostpruningMethodOptions::e_null:
                    {
                        tmp->add(std::make_shared<NullPruningMethod<IncrementalTaskAllocationNode>>());
                        break;
                    }
                    default:
                    {
                        throw createLogicError("Unknown postpruning method");
                    }
                }
            }
            postpruning = std::move(tmp);
        }
        // endregion

        // region scheduler
        std::function<std::shared_ptr<SchedulerBase>(const std::shared_ptr<const SchedulerProblemInputs>&)>
            create_scheduler_function;
        switch(m_builder_options.scheduler)
        {
            case ItagsBuilderOptions::SchedulerOptions::e_deterministic_milp:
            {
                create_scheduler_function =
                    [](const std::shared_ptr<const SchedulerProblemInputs>& scheduler_problem_inputs)
                    -> std::shared_ptr<SchedulerBase>
                {
                    return std::make_shared<DeterministicMilpScheduler>(scheduler_problem_inputs);
                };
                break;
            }
            case ItagsBuilderOptions::SchedulerOptions::e_monolithic_stochastic_milp:
            {
                create_scheduler_function =
                    [](const std::shared_ptr<const SchedulerProblemInputs>& scheduler_problem_inputs)
                    -> std::shared_ptr<SchedulerBase>
                {
                    return std::make_shared<MonolithicStochasticMilpScheduler>(scheduler_problem_inputs);
                };
                break;
            }
            case ItagsBuilderOptions::SchedulerOptions::e_benders_stochastic_milp:
            {
                throw createLogicError("Benders has been deprecated");
                //                                create_scheduler_function =
                //                                    [](const std::shared_ptr<const SchedulerProblemInputs>&
                //                                    scheduler_problem_inputs)
                //                                    -> std::shared_ptr<SchedulerBase>
                //                                {
                //                                    return
                //                                    std::make_shared<BendersStochasticMilpScheduler>(scheduler_problem_inputs);
                //                                };
                break;
            }
            case ItagsBuilderOptions::SchedulerOptions::e_benders_parallel_stochastic_milp:
            {
                throw createLogicError("Benders Parallel has been deprecated");
                //                                create_scheduler_function =
                //                                    [](const std::shared_ptr<const SchedulerProblemInputs>&
                //                                    scheduler_problem_inputs)
                //                                    -> std::shared_ptr<SchedulerBase>
                //                                {
                //                                    return
                //                                    std::make_shared<BendersParallelStochasticMilpScheduler>(scheduler_problem_inputs);
                //                                };
                break;
            }
            case ItagsBuilderOptions::SchedulerOptions::e_heuristic_approximation_stochastic_milp:
            {
                create_scheduler_function =
                    [](const std::shared_ptr<const SchedulerProblemInputs>& scheduler_problem_inputs)
                    -> std::shared_ptr<SchedulerBase>
                {
                    return std::make_shared<HeuristicApproximationStochasticScheduler>(scheduler_problem_inputs);
                };
                break;
            }
            case ItagsBuilderOptions::SchedulerOptions::e_gnn_heuristic_approximation_stochastic_milp:
            {
                create_scheduler_function =
                    [](const std::shared_ptr<const SchedulerProblemInputs>& scheduler_problem_inputs)
                    -> std::shared_ptr<SchedulerBase>
                {
                    return std::make_shared<HeuristicApproximationStochasticScheduler>(
                        scheduler_problem_inputs,
                        [](const std::shared_ptr<const grstapse::SchedulerProblemInputs>& problem_inputs)
                        {
                            return std::make_shared<grstapse::GnnScenarioSelector>(problem_inputs);
                        });
                };
                break;
            }
        }
        // endregion

        // region heuristic
        std::shared_ptr<HeuristicBase<IncrementalTaskAllocationNode>> heuristic;
        switch(m_builder_options.heuristic)
        {
            case ItagsBuilderOptions::HeuristicOptions::e_tetaq:
            {
                if(previous_failure_pruning_method)
                {
                    auto apr = std::make_shared<AllocationPercentageRemaining>(problem_inputs);
                    auto nsq = std::make_shared<NormalizedScheduleQuality>(
                        problem_inputs,
                        create_scheduler_function,
                        [previous_failure_pruning_method](const std::shared_ptr<const SchedulerResult>& result)
                        {
                            previous_failure_pruning_method->addFailureReason(result->failureReason());
                        });
                    heuristic = std::make_shared<TimeExtendedTaskAllocationQuality>(problem_inputs,
                                                                                    m_builder_options.alpha,
                                                                                    apr,
                                                                                    nsq);
                }
                else
                {
                    auto apr  = std::make_shared<AllocationPercentageRemaining>(problem_inputs);
                    auto nsq  = std::make_shared<NormalizedScheduleQuality>(problem_inputs, create_scheduler_function);
                    heuristic = std::make_shared<TimeExtendedTaskAllocationQuality>(problem_inputs,
                                                                                    m_builder_options.alpha,
                                                                                    apr,
                                                                                    nsq);
                }
                break;
            }
            case ItagsBuilderOptions::HeuristicOptions::e_nsq:
            {
                if(previous_failure_pruning_method)
                {
                    heuristic = std::make_shared<NormalizedScheduleQuality>(
                        problem_inputs,
                        create_scheduler_function,
                        [previous_failure_pruning_method](const std::shared_ptr<const SchedulerResult>& result)
                        {
                            previous_failure_pruning_method->addFailureReason(result->failureReason());
                        });
                }
                else
                {
                    heuristic = std::make_shared<NormalizedScheduleQuality>(problem_inputs, create_scheduler_function);
                }
                break;
            }
            case ItagsBuilderOptions::HeuristicOptions::e_apr:
            {
                heuristic = std::make_shared<AllocationPercentageRemaining>(problem_inputs);
                break;
            }
            default:
            {
                throw createLogicError("Unknown heuristic");
            }
        }
        // endregion

        // region successor generator
        std::shared_ptr<SuccessorGeneratorBase<IncrementalTaskAllocationNode>> successor_generator;
        switch(m_builder_options.successor_generator)
        {
            case ItagsBuilderOptions::SuccessorGeneratorOptions::e_increment:
            {
                successor_generator = std::make_shared<IncrementalAllocationGenerator>(problem_inputs);
                break;
            }
            default:
            {
                throw createLogicError("Unknown successor generator");
            }
        }
        // endregion

        std::shared_ptr<GoalCheckBase<IncrementalTaskAllocationNode>> goal_check;
        // region goal check
        switch(m_builder_options.goal_check)
        {
            case ItagsBuilderOptions::GoalCheckOptions::e_zero_apr:
            {
                goal_check = std::make_shared<ZeroAprCheck>(problem_inputs);
                break;
            }
            default:
            {
                throw createLogicError("Unknown goal check");
            }
        }
        // endregion

        std::shared_ptr<MemoizationBase<IncrementalTaskAllocationNode>> memoization;
        // region memoization
        switch(m_builder_options.memoization)
        {
            case ItagsBuilderOptions::MemoizationOptions::e_null:
            {
                memoization = std::make_shared<NullMemoization<IncrementalTaskAllocationNode>>();
                break;
            }
            case ItagsBuilderOptions::MemoizationOptions::e_hash:
            {
                memoization = std::make_shared<HashMemoization<IncrementalTaskAllocationNode>>();
                break;
            }
            default:
            {
                throw createLogicError("Unknown memoization");
            }
        }
        // endregion

        return std::make_shared<Itags>(problem_inputs,
                                       heuristic,
                                       successor_generator,
                                       goal_check,
                                       memoization,
                                       prepruning,
                                       postpruning,
                                       m_builder_options.use_reverse);
    }
}  // namespace grstapse