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

// region Includes
// Global
#include <memory>
#include <mutex>
// External
#include <gurobi_c++.h>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/update_model_result.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    class ParametersBase;
    class FailureReason;
    class MilpSolverResult;
    // endregion

    //! \brief Abstract base class for algorithms that use a MILP formulation
    class MilpSolverBase : public Noncopyable
    {
       public:
        // region Special Member Functions
        MilpSolverBase()                          = delete;
        MilpSolverBase(const MilpSolverBase&)     = delete;
        MilpSolverBase(MilpSolverBase&&) noexcept = default;
        virtual ~MilpSolverBase();
        MilpSolverBase& operator=(const MilpSolverBase&) = delete;
        MilpSolverBase& operator=(MilpSolverBase&&) = delete;
        // endregion

        //! \returns The number of times the MILP optimization was run
        [[nodiscard]] inline unsigned int numIterations() const;

        //! \brief Solves a MILP problem
        [[nodiscard]] std::shared_ptr<MilpSolverResult> solveMilp(
            const std::shared_ptr<const ParametersBase>& parameters);

        /*!
         * \brief Resolves the MILP assuming that something has been updated
         */
        [[nodiscard]] std::shared_ptr<MilpSolverResult> resolve(bool reset = false);

        /*!
         * \brief Resolves the MILP assuming that something has been updated
         */
        [[nodiscard]] std::shared_ptr<MilpSolverResult> resolve(const std::shared_ptr<MilpSolverResult>& result,
                                                                bool reset = false);

        /*!
         * Builds a MILP model
         *
         * \param parameters A set of parameters for the MILP model
         *
         * \returns The model
         */
        [[nodiscard]] virtual std::shared_ptr<MilpSolverResult> createModel(
            const std::shared_ptr<const ParametersBase>& parameters);

        [[nodiscard]] inline std::shared_ptr<GRBModel> model();

        //! Clears and removes all environments
        static void clearEnvironments();

        static void checkEnvironmentErrors();

       protected:
        /*!
         * Sets up the data to be used to generate the model
         *
         * \returns Whether it was successful
         */
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> setupData() = 0;

        //! Set parameters for the MILP \p model
        virtual void setParameters(GRBModel& model, const std::shared_ptr<const ParametersBase>& parameters);

        //! Add variables to the MILP \p model
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createVariables(GRBModel& model) = 0;

        //! Add an objective to the MILP \p model
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createObjective(GRBModel& model) = 0;

        //! Add constraints to the MILP \p model
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createConstraints(GRBModel& model) = 0;

        /*!
         * Updates model after each run (Default: does nothing)
         *
         * \param model
         *
         * \returns The result of attempting to update
         */
        [[nodiscard]] virtual UpdateModelResult updateModel(GRBModel& model);

        //! Constructor
        explicit MilpSolverBase(bool benders_decomposition = false);

        /*!
         * \brief Derivative of GRBCallback to run the BendersDecompositionBase::makeCuts
         */
        class BendersCallback : public GRBCallback
        {
           public:
            explicit BendersCallback(MilpSolverBase* milp_solver, const std::shared_ptr<GRBModel>& model);

            //! \copydoc GRBCallback
            void callback() override;

            /*!
             * \brief Add a lazy constraint to the MIP model
             *
             * \note Only available when the where member variable is equal to GRB_CB_MIPNODE or GRB_CB_MIPSOL
             */
            using GRBCallback::addLazy;
            /*!
             * \brief Retrieve values from the current solution vector
             *
             * \note Only available when the where member variable is equal to GRB_CB_MIPSOL or GRB_CB_MULTIOBJ.
             */
            using GRBCallback::getSolution;

           private:
            MilpSolverBase* m_milp_solver;
            std::shared_ptr<GRBModel> m_model;
        };

        /*!
         * Make cuts to the model
         *
         * \param callback
         */
        virtual void makeCuts(BendersCallback& callback);

        /*!
         * \returns An environment from the environment pool (A new one may be created if one is not available)
         */
        [[nodiscard]] GRBEnv& getEnvironment();

        bool m_return_feasible_on_timeout;
        bool m_benders_decomposition;
        std::unique_ptr<BendersCallback> m_benders_callback;
        std::shared_ptr<GRBModel> m_model;
        unsigned int m_num_iterations;
        int m_environment_index;

        static std::mutex s_environment_lock;
        static std::vector<GRBEnv> s_environment_pool;
        static std::vector<bool> s_environment_taken;
    };

    // Inline Functions
    unsigned int MilpSolverBase::numIterations() const
    {
        return m_num_iterations;
    }

    std::shared_ptr<GRBModel> MilpSolverBase::model()
    {
        return m_model;
    }

}  // namespace grstapse