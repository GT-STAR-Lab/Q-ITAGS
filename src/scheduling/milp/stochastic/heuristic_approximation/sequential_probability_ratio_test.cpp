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
#include "grstapse/scheduling/milp/stochastic/heuristic_approximation/sequential_probability_ratio_test.hpp"

// Global
#include <cmath>
// Local
#include "grstapse/common/utilities/logger.hpp"

namespace grstapse
{
    SequentialProbabilityRatioTest::SequentialProbabilityRatioTest(float p0,
                                                                   float p1,
                                                                   float type1_error,
                                                                   float type2_error)
        : m_p0(p0)
        , m_p1(p1)
        , m_type1_error(type1_error)
        , m_type2_error(type2_error)
    {
        m_denominator = log(m_p1 / m_p0) - log((1.0f - m_p1) / (1.0f - m_p0));
        m_second_term = log((1.0f - m_p0) / (1.0f - m_p1)) / m_denominator;

        // accept H0 if the log-likelihood <= m_a
        m_acceptance_first_term = log(m_type2_error / (1.0f - m_type1_error)) / m_denominator;
        // reject H0 (and hence accept H1) if the log-likelihood >= m_b
        m_rejection_first_term = log((1.0f - m_type2_error) / m_type1_error) / m_denominator;

        // The minimum number of samples for acceptance is computed by assuming that all samples are "good"
        m_min_samples_acceptance = static_cast<unsigned int>(ceilf(-m_acceptance_first_term / m_second_term));
        // The minimum number of samples for rejection is computed by assuming that all samples are "bad"
        m_min_samples_rejection = static_cast<unsigned int>(ceilf(m_rejection_first_term / (1.0f - m_second_term)));
    }

    bool SequentialProbabilityRatioTest::run(float reference_value,
                                             unsigned int max_num_samples,
                                             cppcoro::generator<float> samples) const
    {
        if(max_num_samples < m_min_samples_acceptance && max_num_samples < m_min_samples_rejection)
        {
            Logger::warn("The number of samples in G ({0:d}) is less than the minimum number of samples for acceptance "
                         "({1:d}) and the minimum number of samples for rejection ({2:d})",
                         max_num_samples,
                         m_min_samples_acceptance,
                         m_min_samples_rejection);
            return false;
        }
        else if(max_num_samples < m_min_samples_acceptance)
        {
            Logger::warn(
                "The number of samples in G ({0:d}) is less than the minimum number of samples for acceptance ({1:d})",
                max_num_samples,
                m_min_samples_acceptance);
        }
        else if(max_num_samples < m_min_samples_rejection)
        {
            Logger::warn(
                "The number of samples in G ({0:d}) is less than the minimum number of samples for rejection ({1:d})",
                max_num_samples,
                m_min_samples_acceptance);
        }

        float bad_samples       = 0;
        float inspected_samples = 0;

        const float max_samples_acceptance_number = getAcceptanceNumber(max_num_samples);

        for(const float value: samples)
        {
            if(value > reference_value)
            {
                ++bad_samples;
            }
            ++inspected_samples;

            if(inspected_samples < m_min_samples_acceptance and inspected_samples < m_min_samples_rejection)
            {
                continue;
            }

            if(bad_samples >= getRejectionNumber(inspected_samples) or bad_samples >= max_samples_acceptance_number)
            {
                Logger::warn("SPRT failed due to too many bad samples (inspected: {}).", inspected_samples);
                return false;
            }

            if(bad_samples <= getAcceptanceNumber(inspected_samples))
            {
                Logger::info("SPRT passed (inspected: {}).", inspected_samples);
                return true;
            }
        }

        Logger::warn("SPRT has run out of samples. Returning false.");
        return false;
    }

    float SequentialProbabilityRatioTest::getAcceptanceNumber(float inspected_samples) const
    {
        return m_acceptance_first_term + inspected_samples * m_second_term;
    }

    float SequentialProbabilityRatioTest::getRejectionNumber(float inspected_samples) const
    {
        return m_rejection_first_term + inspected_samples * m_second_term;
    }
}  // namespace grstapse