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
// External
#include <nlohmann/json.hpp>
// Local
#include "grstapse/common/utilities/noncopyable.hpp"
#include "grstapse/common/utilities/timer.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    class SchedulerProblemInputs;
    class SchedulerParameters;
    class SchedulerResult;
    // endregion

    //! \brief Abstract base class for a scheduling algorithm
    class SchedulerBase : public Noncopyable
    {
       public:
        // region Special Member Functions
        SchedulerBase()                         = delete;
        SchedulerBase(const SchedulerBase&)     = delete;
        SchedulerBase(SchedulerBase&&) noexcept = default;
        virtual ~SchedulerBase()                = default;
        SchedulerBase& operator=(const SchedulerBase&) = delete;
        SchedulerBase& operator=(SchedulerBase&&) noexcept = default;
        // endregion

        /*!
         * \brief Solves the scheduling problem
         *
         * \returns The result of attempting to solveMilp the problem
         */
        [[nodiscard]] std::shared_ptr<const SchedulerResult> solve();

        //! \returns The number of time that scheduling has failed
        [[nodiscard]] static unsigned int numFailures();

       protected:
        /*!
         * \brief Constructor
         *
         * \param problem_inputs The inputs for a scheduling problem
         * \param parameters The parameters for configuring the scheduling algorithm
         */
        explicit SchedulerBase(const std::shared_ptr<const SchedulerProblemInputs>& problem_inputs);

        /*!
         * \brief Solves the scheduling problem
         *
         * \returns The result of attempting to solveMilp the problem
         */
        [[nodiscard]] virtual std::shared_ptr<const SchedulerResult> computeSchedule() = 0;

        std::shared_ptr<const SchedulerProblemInputs> m_problem_inputs;

        static unsigned int s_num_failures;
    };

    /*!
     * Concept to force a template parameter to derive from SchedulerBase
     *
     * \tparam T A derivative of SchedulerBase
     */
    template <typename T>
    concept SchedulerDeriv = std::derived_from<T, SchedulerBase>;
}  // namespace grstapse