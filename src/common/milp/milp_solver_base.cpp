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
#include "grstapse/common/milp/milp_solver_base.hpp"

// Local
#include "grstapse/common/milp/milp_infeasible.hpp"
#include "grstapse/common/milp/milp_solver_result.hpp"
#include "grstapse/common/milp/milp_timeout.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/failure_reason.hpp"
#include "grstapse/common/utilities/logger.hpp"
#include "grstapse/parameters/parameters_base.hpp"

namespace grstapse
{
    std::mutex MilpSolverBase::s_environment_lock          = std::mutex();
    std::vector<GRBEnv> MilpSolverBase::s_environment_pool = std::vector<GRBEnv>();
    std::vector<bool> MilpSolverBase::s_environment_taken  = std::vector<bool>();

    MilpSolverBase::MilpSolverBase(bool benders_decomposition)
        : m_return_feasible_on_timeout(false)
        , m_benders_decomposition(benders_decomposition)
        , m_benders_callback(nullptr)
        , m_num_iterations(0)
        , m_environment_index(-1)
    {}

    MilpSolverBase::~MilpSolverBase()
    {
        std::lock_guard<std::mutex> lock(s_environment_lock);
        if(m_environment_index >= 0)
        {
            s_environment_taken[m_environment_index] = false;
        }
    }

    std::shared_ptr<MilpSolverResult> MilpSolverBase::solveMilp(const std::shared_ptr<const ParametersBase>& parameters)
    {
        std::shared_ptr<MilpSolverResult> result = createModel(parameters);
        if(result->failure())
        {
            return result;
        }
        if(m_benders_decomposition)
        {
            m_benders_callback = std::make_unique<BendersCallback>(this, m_model);
            m_model->setCallback(m_benders_callback.get());
        }

        while(true)
        {
            result->incrementNumIterations();
            result = resolve(result);
            if(result->failure())
            {
                return result;
            }

            UpdateModelResult update_model_result = updateModel(*m_model);
            switch(update_model_result.type())
            {
                case UpdateModelResultType::e_no_update:
                {
                    return result;
                }
                case UpdateModelResultType::e_failure:
                {
                    return std::make_shared<MilpSolverResult>(update_model_result.failureReason(),
                                                              result->numIterations());
                }
                // Loop
                case UpdateModelResultType::e_updated:
                {
                    break;
                }
            }
        }
    }

    std::shared_ptr<MilpSolverResult> MilpSolverBase::resolve(bool reset)
    {
        return resolve(std::make_shared<MilpSolverResult>(m_model), reset);
    }

    std::shared_ptr<MilpSolverResult> MilpSolverBase::resolve(const std::shared_ptr<MilpSolverResult>& result,
                                                              bool reset)
    {
        if(reset)
        {
            m_model->reset();
        }
        m_model->update();
        m_model->optimize();

        switch(m_model->get(GRB_IntAttr_Status))
        {
            case GRB_OPTIMAL:
            {
                break;
            }
            case GRB_SUBOPTIMAL:
            {
                Logger::warn("A suboptimal solution was found for the optimization");
                break;
            }
            case GRB_INFEASIBLE:
            {
                Logger::warn("Optimization model determined to be infeasible");
                return std::make_shared<MilpSolverResult>(std::make_shared<MilpInfeasible>(), result->numIterations());
            }
            case GRB_TIME_LIMIT:
            {
                if(m_return_feasible_on_timeout)
                {
                    if(m_model->get(GRB_IntAttr_SolCount) > 0)
                    {
                        Logger::warn("Optimization timed out. Feasible solutions available.");
                        m_model->set(GRB_IntParam_SolutionNumber, 0);
                        break;
                    }
                    else
                    {
                        Logger::warn("Optimization timed out. No feasible solution.");
                        return std::make_shared<MilpSolverResult>(std::make_shared<MilpTimeout>(),
                                                                  result->numIterations());
                    }
                }
                else
                {
                    Logger::warn("Optimization timed out");
                    return std::make_shared<MilpSolverResult>(std::make_shared<MilpTimeout>(), result->numIterations());
                }
            }
            case GRB_UNBOUNDED:
            {
                Logger::warn("Optimization model determined to be unbounded");
                return std::make_shared<MilpSolverResult>(std::make_shared<MilpInfeasible>(), result->numIterations());
            }
            case GRB_INF_OR_UNBD:
            {
                Logger::warn("Optimization model determined to be infeasible or unbounded");
                return std::make_shared<MilpSolverResult>(std::make_shared<MilpInfeasible>(), result->numIterations());
            }
        }
        return result;
    }

    std::shared_ptr<MilpSolverResult> MilpSolverBase::createModel(
        const std::shared_ptr<const ParametersBase>& parameters)
    {
        GRBEnv& env = getEnvironment();
        m_model     = std::make_shared<GRBModel>(env);
        setParameters(*m_model, parameters);

        if(std::shared_ptr<const FailureReason> failure_reason = setupData(); failure_reason)
        {
            return std::make_shared<MilpSolverResult>(failure_reason);
        }
        if(std::shared_ptr<const FailureReason> failure_reason = createVariables(*m_model); failure_reason)
        {
            return std::make_shared<MilpSolverResult>(failure_reason);
        }
        if(std::shared_ptr<const FailureReason> failure_reason = createObjective(*m_model); failure_reason)
        {
            return std::make_shared<MilpSolverResult>(failure_reason);
        }
        if(std::shared_ptr<const FailureReason> failure_reason = createConstraints(*m_model); failure_reason)
        {
            return std::make_shared<MilpSolverResult>(failure_reason);
        }

        m_model->update();
        return std::make_shared<MilpSolverResult>(m_model);
    }

    UpdateModelResult MilpSolverBase::updateModel(GRBModel& model)
    {
        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    void MilpSolverBase::makeCuts(BendersCallback& callback)
    {
        // Default is blank
    }

    void MilpSolverBase::setParameters(GRBModel& model, const std::shared_ptr<const ParametersBase>& parameters)
    {
        if(parameters->contains(constants::k_return_feasible_on_timeout) &&
           parameters->get<bool>(constants::k_return_feasible_on_timeout))
        {
            m_return_feasible_on_timeout = true;
            model.set(GRB_IntParam_PoolSolutions, 1);  // Only need one
        }

        if(parameters->contains(constants::k_milp_timeout) && parameters->get<float>(constants::k_milp_timeout) > 0.0f)
        {
            model.set(GRB_DoubleParam_TimeLimit,
                      static_cast<double>(parameters->get<float>(constants::k_milp_timeout)));
        }

        if(parameters->contains(constants::k_mip_gap) && parameters->get<float>(constants::k_mip_gap) > 0.0f)
        {
            model.set(GRB_DoubleParam_MIPGap, static_cast<double>(parameters->get<float>(constants::k_mip_gap)));
        }

        if(parameters->contains(constants::k_heuristic_time) &&
           parameters->get<float>(constants::k_heuristic_time) > 0.0f)
        {
            model.set(GRB_DoubleParam_Heuristics,
                      static_cast<double>(parameters->get<float>(constants::k_heuristic_time)));
        }

        if(parameters->contains(constants::k_method) && parameters->get<int>(constants::k_method) >= 0)
        {
            model.set(GRB_IntParam_Method, parameters->get<int>(constants::k_method));
        }

        if(m_benders_decomposition)
        {
            model.set(GRB_IntParam_LazyConstraints, 1);
        }
    }

    GRBEnv& MilpSolverBase::getEnvironment()
    {
        std::lock_guard<std::mutex> lock(s_environment_lock);
        if(s_environment_pool.empty())
        {
            GRBEnv& env = s_environment_pool.emplace_back(true);
            env.set(GRB_IntParam_LogToConsole, 0);
            env.start();
        }
        return s_environment_pool.front();
        //        // Find the first untaken environment
        //        for(unsigned int i = 0; i < s_environment_taken.size(); ++i)
        //        {
        //            if(not s_environment_taken[i])
        //            {
        //                m_environment_index    = i;
        //                s_environment_taken[i] = true;
        //                return s_environment_pool[i];
        //            }
        //        }
        //
        //        m_environment_index = s_environment_taken.size();
        //        s_environment_taken.emplace_back(true);
        //
        //        GRBEnv& env = s_environment_pool.emplace_back(true);
        //        env.set(GRB_IntParam_LogToConsole, 0);
        //        env.start();
        //        return env;
    }

    void MilpSolverBase::clearEnvironments()
    {
        std::lock_guard<std::mutex> lock(s_environment_lock);
        if(std::any_of(s_environment_taken.begin(),
                       s_environment_taken.end(),
                       [](bool b) -> bool  // Hopefully the compiler is smart about this...
                       {
                           return b;
                       }))
        {
            throw createLogicError("Attempting to clear gurobi environments while some are still in use");
        }
        s_environment_pool.clear();
        s_environment_taken.clear();
    }

    MilpSolverBase::BendersCallback::BendersCallback(MilpSolverBase* milp_solver,
                                                     const std::shared_ptr<GRBModel>& model)
        : m_milp_solver(milp_solver)
        , m_model(model)
    {}

    void MilpSolverBase::BendersCallback::callback()
    {
        if(where == GRB_CB_MIPSOL)
        {
            m_milp_solver->makeCuts(*this);
        }
    }

    void MilpSolverBase::checkEnvironmentErrors()
    {
        for(GRBEnv& env: s_environment_pool)
        {
            Logger::warn(env.getErrorMsg());
        }
    }
}  // namespace grstapse