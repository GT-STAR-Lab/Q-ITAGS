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
#include <tuple>
#include <vector>
// Local
#include "grstapse/problem_inputs/itags_problem_inputs.hpp"

namespace grstapse
{
    /*!
     * \brief Container for the inputs to a scheduling problem
     */
    class SchedulerProblemInputs : public ProblemInputs
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param problem_inputs
         * \param allocation
         */
        explicit SchedulerProblemInputs(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                        const Eigen::MatrixXf& allocation);

        /*!
         * \brief Constructor
         *
         * \param problem_inputs
         * \param allocation
         */
        explicit SchedulerProblemInputs(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs,
                                        Eigen::MatrixXf&& allocation);

        /*!
         * Throws an exception if the output from task planning isn't valid
         *
         * Reasons it could be invalid:
         * - mutex constraint uses a index that is out of range of the number of plan tasks
         * - precedence constraint uses a index that is out of range of the number of plan tasks
         */
        void validate() const;

        /*!
         * \returns A view of the coalition of robots assigned to task \p task_nr
         *
         * \note The typename for the view is unwieldy so we use auto. This requires the implementation of the function
         *       to be in the header
         */
        [[nodiscard]] CoalitionView coalition(unsigned int task_nr) const;

        /*!
         * \returns A view of the coalition of robots assigned to both tasks \p i and \p j
         *
         * \note The typename for the view is unwieldy so we use auto. This requires the implementation of the function
         *       to be in the header
         */
        [[nodiscard]] CoalitionView transitionCoalition(unsigned int i, unsigned int j) const;

        [[nodiscard]] inline const std::shared_ptr<const ItagsProblemInputs>& itagsProblemInputs() const;

        // Output from Task Allocation
        [[nodiscard]] inline const Eigen::MatrixXf& allocation() const;
        [[nodiscard]] virtual inline const std::set<std::pair<unsigned int, unsigned int>>& mutexConstraints() const;
        [[nodiscard]] inline float scheduleBestMakespan() const;
        [[nodiscard]] inline float scheduleWorstMakespan() const;

        // Output from Task Planning
        [[nodiscard]] inline PlanView planTasks() const;
        [[nodiscard]] inline const std::shared_ptr<const Task>& planTask(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfPlanTasks() const;
        [[nodiscard]] virtual inline const std::set<std::pair<unsigned int, unsigned int>>& precedenceConstraints()
            const;

        // Module Parameters
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& schedulerParameters() const;

        // Problem Inputs
        //// Tasks
        //// Robots
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Robot>>& robots() const;
        [[nodiscard]] inline const std::shared_ptr<const Robot>& robot(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfRobots() const;
        //// Species
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Species>>& multipleSpecies() const;
        [[nodiscard]] inline const std::shared_ptr<const Species>& individualSpecies(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfSpecies() const;
        //// Motion Planners
        [[nodiscard]] inline const std::vector<std::shared_ptr<MotionPlannerBase>>& motionPlanners() const;
        [[nodiscard]] inline const std::shared_ptr<MotionPlannerBase>& motionPlanner(unsigned int index) const;

       protected:
        // From task allocation (order matters)
        std::set<std::pair<unsigned int, unsigned int>> m_mutex_constraints;  //!< Must be before m_allocation
        Eigen::MatrixXf m_allocation;                                         //!< Must be after m_mutex_constraints
        // todo deadlines?

        std::shared_ptr<const ItagsProblemInputs> m_itags_problem_inputs;

        friend class nlohmann::adl_serializer<std::shared_ptr<SchedulerProblemInputs>>;
    };

    // Inline functions
    const Eigen::MatrixXf& SchedulerProblemInputs::allocation() const
    {
        return m_allocation;
    }
    const std::set<std::pair<unsigned int, unsigned int>>& SchedulerProblemInputs::mutexConstraints() const
    {
        return m_mutex_constraints;
    }
    const std::shared_ptr<const ItagsProblemInputs>& SchedulerProblemInputs::itagsProblemInputs() const
    {
        return m_itags_problem_inputs;
    }
    PlanView SchedulerProblemInputs::planTasks() const
    {
        return m_itags_problem_inputs->planTasks();
    }
    const std::shared_ptr<const Task>& SchedulerProblemInputs::planTask(unsigned int index) const
    {
        return m_itags_problem_inputs->planTask(index);
    }
    unsigned int SchedulerProblemInputs::numberOfPlanTasks() const
    {
        return m_itags_problem_inputs->numberOfPlanTasks();
    }
    const std::set<std::pair<unsigned int, unsigned int>>& SchedulerProblemInputs::precedenceConstraints() const
    {
        return m_itags_problem_inputs->precedenceConstraints();
    }
    const std::shared_ptr<const ParametersBase>& SchedulerProblemInputs::schedulerParameters() const
    {
        return m_itags_problem_inputs->schedulerParameters();
    }
    const std::vector<std::shared_ptr<const Robot>>& SchedulerProblemInputs::robots() const
    {
        return m_itags_problem_inputs->robots();
    }
    const std::shared_ptr<const Robot>& SchedulerProblemInputs::robot(unsigned int index) const
    {
        return m_itags_problem_inputs->robot(index);
    }
    unsigned int SchedulerProblemInputs::numberOfRobots() const
    {
        return m_itags_problem_inputs->numberOfRobots();
    }
    const std::vector<std::shared_ptr<const Species>>& SchedulerProblemInputs::multipleSpecies() const
    {
        return m_itags_problem_inputs->multipleSpecies();
    }
    const std::shared_ptr<const Species>& SchedulerProblemInputs::individualSpecies(unsigned int index) const
    {
        return m_itags_problem_inputs->individualSpecies(index);
    }
    unsigned int SchedulerProblemInputs::numberOfSpecies() const
    {
        return m_itags_problem_inputs->numberOfSpecies();
    }
    const std::vector<std::shared_ptr<MotionPlannerBase>>& SchedulerProblemInputs::motionPlanners() const
    {
        return m_itags_problem_inputs->motionPlanners();
    }
    const std::shared_ptr<MotionPlannerBase>& SchedulerProblemInputs::motionPlanner(unsigned int index) const
    {
        return m_itags_problem_inputs->motionPlanner(index);
    }
    float SchedulerProblemInputs::scheduleBestMakespan() const
    {
        return m_itags_problem_inputs->scheduleBestMakespan();
    }
    float SchedulerProblemInputs::scheduleWorstMakespan() const
    {
        return m_itags_problem_inputs->scheduleWorstMakespan();
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<grstapse::SchedulerProblemInputs>>
    {
        static std::shared_ptr<grstapse::SchedulerProblemInputs> from_json(const nlohmann::json& j);
    };

}  // namespace nlohmann