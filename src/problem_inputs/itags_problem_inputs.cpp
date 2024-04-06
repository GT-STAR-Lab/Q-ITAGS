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
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"

// External
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/custom_views.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_extension.hpp"
#include "grstapse/common/utilities/json_tree_factory.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/common/utilities/ranges_extension.hpp"
#include "grstapse/common/utilities/std_extension.hpp"
#include "grstapse/common/utilities/time_keeper.hpp"
#include "grstapse/geometric_planning/configurations/configuration_base.hpp"
#include "grstapse/geometric_planning/environments/ompl_environment.hpp"
#include "grstapse/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "grstapse/parameters/parameters_factory.hpp"
#include "grstapse/problem_inputs/grstaps_problem_inputs.hpp"
#include "grstapse/problem_inputs/scheduler_problem_inputs.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_milp_scheduler.hpp"
#include "grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp"
#include "grstapse/scheduling/scheduler_result.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task.hpp"
#include "grstapse/task_allocation/itags/robot_traits_matrix_reduction.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"
#include "grstapse/task_planning/sas/sas_action.hpp"

namespace grstapse
{
    ItagsProblemInputs::ItagsProblemInputs(const ThisIsProtectedTag&)
        : m_grstaps_problem_inputs(nullptr)
        , m_schedule_best_makespan(std::numeric_limits<float>::quiet_NaN())
        , m_schedule_worst_makespan(std::numeric_limits<float>::quiet_NaN())
    {}

    ItagsProblemInputs::ItagsProblemInputs(
        const std::shared_ptr<const GrstapsProblemInputs>& problem_inputs,
        const std::vector<unsigned int>& plan_task_indicies,
        const std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints,
        bool use_reverse,
        float max_schedule)
        : m_grstaps_problem_inputs(problem_inputs)
        , m_plan_task_indices(plan_task_indicies)
        , m_use_reverse(use_reverse)
        , m_max_schedule(max_schedule)
    {
        if(problem_inputs != nullptr)
        {
            m_desired_traits_matrix =
                grstapse::desiredTraitsMatrix(planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
            m_linear_coefficient_matrix = grstapse::linearCoefficientMatrix(
                planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
        }
        else
        {
            Logger::warn("GrstapsProblemInput is NULL");
        }

        m_precedence_constraints = addPrecedenceTransitiveConstraints(precedence_constraints);
        m_use_reverse            = false;
        m_max_schedule           = 0;

        computeScheduleBestWorst();
    }

    ItagsProblemInputs::ItagsProblemInputs(
        const std::shared_ptr<const GrstapsProblemInputs>& problem_inputs,
        const std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints,
        bool use_reverse,
        float max_schedule)
        : m_grstaps_problem_inputs(problem_inputs)
        , m_use_reverse(use_reverse)
        , m_max_schedule(max_schedule)
    {
        if(problem_inputs != nullptr)
        {
            m_plan_task_indices =
                ::ranges::views::iota(0u, problem_inputs->numberOfTasks()) | ranges::to<std::vector<unsigned int>>();
            m_desired_traits_matrix =
                grstapse::desiredTraitsMatrix(planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
            m_linear_coefficient_matrix = grstapse::linearCoefficientMatrix(
                planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
        }
        else
        {
            Logger::warn("GrstapsProblemInput is NULL");
        }
        m_precedence_constraints = addPrecedenceTransitiveConstraints(precedence_constraints);
        computeScheduleBestWorst();
    }

    ItagsProblemInputs::ItagsProblemInputs(const std::shared_ptr<const GrstapsProblemInputs>& problem_inputs,
                                           std::set<std::pair<unsigned int, unsigned int>>&& precedence_constraints,
                                           bool use_reverse,
                                           float max_schedule)
        : m_grstaps_problem_inputs(problem_inputs)
        , m_use_reverse(use_reverse)
        , m_max_schedule(max_schedule)
    {
        if(problem_inputs != nullptr)
        {
            m_plan_task_indices =
                ::ranges::views::iota(0u, problem_inputs->numberOfTasks()) | ::ranges::to<std::vector<unsigned int>>();
            m_desired_traits_matrix =
                grstapse::desiredTraitsMatrix(planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
            m_linear_coefficient_matrix = grstapse::linearCoefficientMatrix(
                planTasks() | ::ranges::to<std::vector<std::shared_ptr<const Task>>>());
        }
        else
        {
            Logger::warn("GrstapsProblemInput is NULL");
        }
        m_precedence_constraints = addPrecedenceTransitiveConstraints(precedence_constraints);
        computeScheduleBestWorst();
    }

    void ItagsProblemInputs::validate() const
    {
        unsigned int num_plan_task = numberOfPlanTasks();
        for(const std::pair<unsigned int, unsigned int>& constraint: m_precedence_constraints)
        {
            if(constraint.first >= num_plan_task || constraint.second >= num_plan_task)
            {
                throw createLogicError("Precedence constraint out of range of the number of plan tasks");
            }
        }
    }

    PlanView ItagsProblemInputs::planTasks() const
    {
        return m_plan_task_indices | ::ranges::views::transform(std::function(
                                         [this](unsigned int i) -> const std::shared_ptr<const Task>&
                                         {
                                             return m_grstaps_problem_inputs->task(i);
                                         }));
    }

    const std::shared_ptr<const Task>& ItagsProblemInputs::planTask(unsigned int index) const
    {
        return planTasks()[index];
    }

    std::vector<std::shared_ptr<const Task>> ItagsProblemInputs::loadTasks(
        const nlohmann::json& j,
        const std::shared_ptr<const GrstapsProblemInputs>& grstaps_problem_inputs)
    {
        if(!j.is_array())
        {
            throw createLogicError("''tasks' must be an array");
        }

        std::vector<std::shared_ptr<const Task>> tasks;
        tasks.reserve(j.size());
        for(const nlohmann::json& task_j: j)
        {
            std::string name = "";
            if(const auto& name_j_itr = task_j.find(constants::k_name); name_j_itr != task_j.end())
            {
                name = *name_j_itr;
            }
            const float duration                              = task_j.at(constants::k_duration);
            const Eigen::VectorXf desired_traits              = task_j.at(constants::k_desired_traits);
            const Eigen::VectorXf linear_quality_coefficients = task_j.at(constants::k_linear_quality_coefficients);
            const nlohmann::json& initial_configuration_j     = task_j.at(constants::k_initial_configuration);
            const nlohmann::json& terminal_configuration_j    = task_j.at(constants::k_terminal_configuration);

            const std::shared_ptr<const ConfigurationBase> initial_configuration =
                initial_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            grstaps_problem_inputs->checkConfiguration(initial_configuration);

            const std::shared_ptr<const ConfigurationBase> terminal_configuration =
                terminal_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            grstaps_problem_inputs->checkConfiguration(terminal_configuration);

            tasks.push_back(std::make_shared<const Task>(std::make_shared<SasAction>(name, duration),
                                                         desired_traits,
                                                         initial_configuration,
                                                         terminal_configuration,
                                                         linear_quality_coefficients));
        }
        return tasks;
    }

    void ItagsProblemInputs::computeScheduleBestWorst()
    {
        // Compute makespan for schedule best
        // TODO(Andrew): turn this into a utility/static function somewhere
        {
            // Create empty allocation matrix
            Eigen::MatrixXf allocation(numberOfPlanTasks(), numberOfRobots());
            allocation.setZero();

            // Pass a shared_ptr that doesn't delete
            auto scheduler_problem_inputs = std::shared_ptr<SchedulerProblemInputs>(
                new SchedulerProblemInputs(std::shared_ptr<ItagsProblemInputs>(this, [](ItagsProblemInputs*) {}),
                                           allocation));
            DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            std::shared_ptr<const SchedulerResult> scheduler_result = scheduler.solve();
            if(scheduler_result->failed())
            {
                throw createLogicError("Schedule best cannot be created. Problem is unsolvable.");
            }
            m_schedule_best_makespan = scheduler_result->schedule()->makespan();
        }

        // Compute makespan for schedule worst
        // TODO(Andrew): turn this into a utility/static function somewhere
        {
            const float slowest_speed = multipleSpecies() |
                                        ::ranges::views::transform(
                                            [](const std::shared_ptr<const Species>& species) -> float
                                            {
                                                return species->speed();
                                            }) |
                                        ranges_ext::min<float>();
            const float longest_path = motionPlanners() |
                                       ::ranges::views::transform(
                                           [](const std::shared_ptr<MotionPlannerBase>& motion_planner) -> float
                                           {
                                               return motion_planner->environment()->longestPath();
                                           }) |
                                       ranges_ext::max<float>();

            const float worst_mp_duration = longest_path / slowest_speed;
            m_schedule_worst_makespan     = planTasks() |
                                        ::ranges::views::transform(
                                            [=](const std::shared_ptr<const Task>& task) -> float
                                            {
                                                return 2.0f * worst_mp_duration + task->staticDuration();
                                            }) |
                                        ranges_ext::sum<float>();
        }
    }
}  // namespace grstapse

namespace nlohmann
{
    std::shared_ptr<grstapse::ItagsProblemInputs>
    adl_serializer<std::shared_ptr<grstapse::ItagsProblemInputs>>::from_json(const nlohmann::json& j)
    {
        auto problem_inputs =
            std::make_shared<grstapse::ItagsProblemInputs>(grstapse::ProblemInputs::s_this_is_protected_tag);
        // Load/Create GRSTAPS stuff
        {
            auto grstaps_problem_inputs =
                std::make_shared<grstapse::GrstapsProblemInputs>(grstapse::ProblemInputs::s_this_is_protected_tag);

            // Load Environments & Motion Planners
            grstaps_problem_inputs->loadMotionPlanners(j.at(grstapse::constants::k_motion_planners));

            // Create Mock tasks
            grstaps_problem_inputs->m_tasks =
                problem_inputs->loadTasks(j.at(grstapse::constants::k_tasks), grstaps_problem_inputs);

            // Load Species
            auto [name_to_species_mapping, num_traits] =
                grstaps_problem_inputs->loadSpecies(j.at(grstapse::constants::k_species));

            // Load Robots
            // Note: cannot use the normal from_json function because the vector of species is needed
            grstaps_problem_inputs->loadRobots(name_to_species_mapping,
                                               num_traits,
                                               j.at(grstapse::constants::k_robots));

            // Load Module Parameters
            {
                grstaps_problem_inputs->m_itags_parameters =
                    grstapse::ParametersFactory::instance().create(grstapse::ParametersFactory::Type::e_search,
                                                                   j.at(grstapse::constants::k_itags_parameters));
                if(j.find(grstapse::constants::k_robot_traits_matrix_reduction) != j.end())
                {
                    grstaps_problem_inputs->m_robot_traits_matrix_reduction =
                        j.at(grstapse::constants::k_robot_traits_matrix_reduction)
                            .get<std::shared_ptr<grstapse::RobotTraitsMatrixReduction>>();
                }
                else
                {
                    grstaps_problem_inputs->m_robot_traits_matrix_reduction =
                        std::make_shared<const grstapse::RobotTraitsMatrixReduction>();
                }
                grstaps_problem_inputs->m_scheduler_parameters =
                    grstapse::ParametersFactory::instance().create(grstapse::ParametersFactory::Type::e_scheduler,
                                                                   j.at(grstapse::constants::k_scheduler_parameters));
                // MP parameters are loaded up above
            }
            problem_inputs->m_grstaps_problem_inputs = grstaps_problem_inputs;
        }

        // Load TP Stuff
        if(j.contains(grstapse::constants::k_plan_task_indices))
        {
            j.at(grstapse::constants::k_plan_task_indices).get_to(problem_inputs->m_plan_task_indices);
        }
        else
        {
            problem_inputs->m_plan_task_indices =
                std::views::iota(0u, problem_inputs->m_grstaps_problem_inputs->numberOfTasks()) |
                ::ranges::to<std::vector<unsigned int>>();
        }

        // Load TP Stuff
        if(j.contains(grstapse::constants::k_plan_task_indices))
        {
            j.at(grstapse::constants::k_plan_task_indices).get_to(problem_inputs->m_plan_task_indices);
        }
        else
        {
            problem_inputs->m_plan_task_indices =
                std::views::iota(0u, problem_inputs->m_grstaps_problem_inputs->numberOfTasks()) |
                ::ranges::to<std::vector<unsigned int>>();
        }

        if(j.contains(grstapse::constants::k_use_reverse))
        {
            j.at(grstapse::constants::k_use_reverse).get_to(problem_inputs->m_use_reverse);
        }
        else
        {
            problem_inputs->m_use_reverse = false;
        }

        if(j.contains(grstapse::constants::k_max_schedule))
        {
            j.at(grstapse::constants::k_max_schedule).get_to(problem_inputs->m_max_schedule);
        }
        else
        {
            problem_inputs->m_max_schedule = 0.0;
        }

        std::set<std::pair<unsigned int, unsigned int>> tmp;
        j.at(grstapse::constants::k_precedence_constraints).get_to(tmp);
        problem_inputs->m_precedence_constraints = grstapse::addPrecedenceTransitiveConstraints(std::move(tmp));
        problem_inputs->m_desired_traits_matrix =
            desiredTraitsMatrix(problem_inputs->m_grstaps_problem_inputs->m_tasks, problem_inputs->m_plan_task_indices);
        problem_inputs->m_linear_coefficient_matrix =
            linearCoefficientMatrix(problem_inputs->m_grstaps_problem_inputs->m_tasks,
                                    problem_inputs->m_plan_task_indices);

        if(j.contains(grstapse::constants::k_best_schedule) && j.contains(grstapse::constants::k_worst_schedule))
        {
            j.at(grstapse::constants::k_best_schedule).get_to(problem_inputs->m_schedule_best_makespan);
            j.at(grstapse::constants::k_worst_schedule).get_to(problem_inputs->m_schedule_worst_makespan);
        }
        else
        {
            problem_inputs->computeScheduleBestWorst();
            grstapse::TimeKeeper::instance().reset(grstapse::constants::k_scheduling_time);
        }

        return problem_inputs;
    }
}  // namespace nlohmann