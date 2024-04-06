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
#include <vector>
// External
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <ompl/base/Goal.h>
// Local
#include "problem_inputs.hpp"

namespace grstapse
{
    // Forward Declarations
    class ConfigurationBase;
    class EnvironmentBase;
    class ItagsProblemInputs;
    class MotionPlannerBase;
    class Robot;
    class RobotTraitsMatrixReduction;
    class SasAction;
    class SchedulerProblemInputs;
    class Species;
    class Task;
    class ParametersBase;
    enum class ConfigurationType : uint8_t;
    enum class OmplStateSpaceType : uint8_t;
    enum class GraphType : uint8_t;

    //! A container for input for a GRSTAPS problem
    class GrstapsProblemInputs : public ProblemInputs
    {
       public:
        /*!
         * \brief Acts as a protected default constructor. Used for json deserialization
         */
        explicit GrstapsProblemInputs(const ThisIsProtectedTag&);

        //! Destructor
        ~GrstapsProblemInputs();

        // Module Parameters
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& fcpopParameters() const;
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& itagsParameters() const;
        [[nodiscard]] inline const std::shared_ptr<const RobotTraitsMatrixReduction>& robotTraitsMatrixReduction()
            const;
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& schedulerParameters() const;

        // Problem Inputs
        //// Tasks
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Task>>& tasks() const;
        [[nodiscard]] inline const std::shared_ptr<const Task>& task(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfTasks() const;
        //// Robots
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Robot>>& robots() const;
        [[nodiscard]] inline const std::shared_ptr<const Robot>& robot(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfRobots() const;
        //// Species
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Species>>& multipleSpecies() const;
        [[nodiscard]] inline const std::shared_ptr<const Species>& individualSpecies(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfSpecies() const;
        //// Team Traits Matrix
        [[nodiscard]] inline const Eigen::MatrixXf& teamTraitsMatrix() const;
        [[nodiscard]] inline unsigned int numberOfTraits() const;
        //// Motion Planners
        [[nodiscard]] inline const std::vector<std::shared_ptr<MotionPlannerBase>>& motionPlanners() const;
        [[nodiscard]] inline const std::shared_ptr<MotionPlannerBase>& motionPlanner(unsigned int index) const;

        //! Checks if \p configuration matches the previously loaded environments
        void checkConfiguration(const std::shared_ptr<const ConfigurationBase>& configuration) const;

       protected:
        //! Load motion planners from json
        void loadMotionPlanners(const nlohmann::json& j);

        //! Merges symbolic actions with non-symbolic information
        void createTasks(const std::vector<std::shared_ptr<SasAction>>& grounded_sas_actions, const nlohmann::json& j);

        //! Loads species from json
        std::pair<std::map<std::string, std::shared_ptr<const Species>>, unsigned int> loadSpecies(
            const nlohmann::json& j);

        //! Loads robots from json
        void loadRobots(const std::map<std::string, std::shared_ptr<const Species>>& name_to_species_mapping,
                        const unsigned int num_traits,
                        const nlohmann::json& j);

        // Module Parameters
        std::shared_ptr<const ParametersBase> m_fcpop_parameters;
        std::shared_ptr<const ParametersBase> m_itags_parameters;
        std::shared_ptr<const RobotTraitsMatrixReduction> m_robot_traits_matrix_reduction;
        std::shared_ptr<const ParametersBase> m_scheduler_parameters;
        // MP parameters are passed directly to the individual motion planners

        // Problem Inputs
        std::vector<std::shared_ptr<const Task>> m_tasks;
        std::vector<std::shared_ptr<const Robot>> m_robots;
        std::vector<std::shared_ptr<const Species>> m_species;
        Eigen::MatrixXf m_team_traits_matrix;
        std::vector<std::shared_ptr<MotionPlannerBase>> m_motion_planners;

        ConfigurationType m_task_configuration_type;
        OmplStateSpaceType m_ompl_state_space_type;
        GraphType m_graph_type;

        friend struct nlohmann::adl_serializer<std::shared_ptr<GrstapsProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<ItagsProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<SchedulerProblemInputs>>;
    };

    // Inlined Functions
    const std::shared_ptr<const ParametersBase>& GrstapsProblemInputs::fcpopParameters() const
    {
        return m_fcpop_parameters;
    }

    const std::shared_ptr<const ParametersBase>& GrstapsProblemInputs::itagsParameters() const
    {
        return m_itags_parameters;
    }

    const std::shared_ptr<const RobotTraitsMatrixReduction>& GrstapsProblemInputs::robotTraitsMatrixReduction() const
    {
        return m_robot_traits_matrix_reduction;
    }

    const std::shared_ptr<const ParametersBase>& GrstapsProblemInputs::schedulerParameters() const
    {
        return m_scheduler_parameters;
    }

    const std::vector<std::shared_ptr<const Task>>& GrstapsProblemInputs::tasks() const
    {
        return m_tasks;
    }

    const std::shared_ptr<const Task>& GrstapsProblemInputs::task(unsigned int index) const
    {
        assert(index < m_tasks.size());
        return m_tasks[index];
    }

    unsigned int GrstapsProblemInputs::numberOfTasks() const
    {
        return m_tasks.size();
    }

    const std::vector<std::shared_ptr<const Robot>>& GrstapsProblemInputs::robots() const
    {
        return m_robots;
    }

    const std::shared_ptr<const Robot>& GrstapsProblemInputs::robot(unsigned int index) const
    {
        assert(index < m_robots.size());
        return m_robots[index];
    }

    unsigned int GrstapsProblemInputs::numberOfRobots() const
    {
        return m_robots.size();
    }

    const std::vector<std::shared_ptr<const Species>>& GrstapsProblemInputs::multipleSpecies() const
    {
        return m_species;
    }

    const std::shared_ptr<const Species>& GrstapsProblemInputs::individualSpecies(unsigned int index) const
    {
        assert(index < m_species.size());
        return m_species[index];
    }

    unsigned int GrstapsProblemInputs::numberOfSpecies() const
    {
        return m_species.size();
    }

    const Eigen::MatrixXf& GrstapsProblemInputs::teamTraitsMatrix() const
    {
        return m_team_traits_matrix;
    }

    unsigned int GrstapsProblemInputs::numberOfTraits() const
    {
        return m_team_traits_matrix.cols();
    }

    const std::vector<std::shared_ptr<MotionPlannerBase>>& GrstapsProblemInputs::motionPlanners() const
    {
        return m_motion_planners;
    }

    const std::shared_ptr<MotionPlannerBase>& GrstapsProblemInputs::motionPlanner(unsigned int index) const
    {
        assert(index < m_motion_planners.size());
        return m_motion_planners[index];
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<grstapse::GrstapsProblemInputs>>
    {
        static std::shared_ptr<grstapse::GrstapsProblemInputs> from_json(const nlohmann::json& j);
    };
}  // namespace nlohmann