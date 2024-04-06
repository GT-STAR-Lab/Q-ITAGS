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
// External
#include <cppcoro/generator.hpp>
// endregion

namespace grstapse
{
    /*!
     * \class SequentialProbabilityRatioTest
     * \cite A. Wald. Sequential Test of Statistical Hypotheses. Ann. Math. Statist. 16(2): 158 (June, 1945)
     */
    class SequentialProbabilityRatioTest
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        SequentialProbabilityRatioTest() = delete;
        //! Copy Constructor
        SequentialProbabilityRatioTest(const SequentialProbabilityRatioTest&) = delete;
        //! Move Constructor
        SequentialProbabilityRatioTest(SequentialProbabilityRatioTest&&) noexcept = default;
        //! Destructor
        ~SequentialProbabilityRatioTest() = default;
        //! Copy Assignment Operator
        SequentialProbabilityRatioTest& operator=(const SequentialProbabilityRatioTest&) = delete;
        //! Move Assignment Operator
        SequentialProbabilityRatioTest& operator=(SequentialProbabilityRatioTest&&) noexcept = default;
        // endregion

        /*!
         * \brief Constructor
         *
         * \param p0 alpha-robustness - small tolerance (defines indifference interval)
         * \param p1 alpha-robustness + small tolerance (defines indifference interval)
         * \param type1_error Type 1 Error (alpha)
         * \param type2_error Type 2 Error (beta)
         */
        explicit SequentialProbabilityRatioTest(float p0,
                                                float p1,
                                                float type1_error = 0.05f,
                                                float type2_error = 0.05f);

        /*!
         * Runs the test
         *
         * \param reference_value The makespan computed by the approximation MILP
         * \param max_num_samples The maximum number of samples
         * \param samples A generator that computes makespans for elements of the G set on the fly
         *
         * \returns true if, according to the SPRT, the probability of obtaining a scenario where the policy execution
         *          would result in a makespan greater than \p makespan is smaller than p0; otherwise false (also
         *          returns false if the test runs out of samples)
         */
        [[nodiscard]] bool run(float reference_value,
                               unsigned int max_num_samples,
                               cppcoro::generator<float> samples) const;

       private:
        //! \returns
        [[nodiscard]] float getAcceptanceNumber(float inspected_samples) const;

        //! \returns
        [[nodiscard]] float getRejectionNumber(float inspected_samples) const;

        float m_p0;           //!< alpha-robustness - small tolerance (defines indifference interval)
        float m_p1;           //!< alpha-robustness + small tolerance (defines indifference interval)
        float m_type1_error;  //!< Type 1 Error (alpha)
        float m_type2_error;  //!< Type 2 Error (beta)

        unsigned int m_min_samples_acceptance;
        unsigned int m_min_samples_rejection;

        float m_second_term;
        float m_denominator;
        float m_acceptance_first_term;  //!< Accept H0 if the log-likelihood <= m_a
        float m_rejection_first_term;   //!< Reject H0 (and hence accept H1) if the log-likelihood >= m_b
    };                                  // class SequentialProbabilityRatioTest
}  // namespace grstapse