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
// Local
#include "grstapse/geometric_planning/motion_planners/complete_sampled_euclidean_graph_motion_planner.hpp"
// endregion

namespace grstapse
{
    // region Forward Declarations
    // endregion

    /*!
     * \class MaskedCompleteSampledEuclideanGraphMotionPlanner
     * \brief
     */
    class MaskedCompleteSampledEuclideanGraphMotionPlanner : public CompleteSampledEuclideanGraphMotionPlanner
    {
       public:
        // region Special Member Functions
        //! Default Constructor
        MaskedCompleteSampledEuclideanGraphMotionPlanner() = delete;
        //! Copy Constructor
        MaskedCompleteSampledEuclideanGraphMotionPlanner(const MaskedCompleteSampledEuclideanGraphMotionPlanner&) =
            delete;
        //! Move Constructor
        MaskedCompleteSampledEuclideanGraphMotionPlanner(MaskedCompleteSampledEuclideanGraphMotionPlanner&&) noexcept =
            default;
        //! Destructor
        ~MaskedCompleteSampledEuclideanGraphMotionPlanner() = default;
        //! Copy Assignment Operator
        MaskedCompleteSampledEuclideanGraphMotionPlanner& operator   =(
            const MaskedCompleteSampledEuclideanGraphMotionPlanner&) = delete;
        //! Move Assignment Operator
        MaskedCompleteSampledEuclideanGraphMotionPlanner& operator       =(
            MaskedCompleteSampledEuclideanGraphMotionPlanner&&) noexcept = default;
        // endregion

        //! Constructor
        explicit MaskedCompleteSampledEuclideanGraphMotionPlanner(
            const std::shared_ptr<const ParametersBase>& parameters,
            const std::shared_ptr<SampledEuclideanGraphEnvironment>& environment);

        void setMask(const std::vector<bool>& mask);

        [[nodiscard]] inline unsigned int numMasked() const;

        [[nodiscard]] inline unsigned int totalNumber() const;

        //! \copydoc SampledEuclideanGraphMotionPlannerBase
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> query(
            unsigned int index,
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
            const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration) override;

        //! \copydoc SampledEuclideanGraphMotionPlannerBase
        [[nodiscard]] bool isMemoized(
            unsigned int index,
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
            const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration) const override;

        //! \copydoc SampledEuclideanGraphMotionPlannerBase
        [[nodiscard]] float durationQuery(
            unsigned int index,
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const EuclideanGraphConfiguration>& initial_configuration,
            const std::shared_ptr<const EuclideanGraphConfiguration>& goal_configuration) override;

       private:
        std::unordered_map<unsigned int, unsigned int> m_indices;
    };  // class MaskedCompleteSampledEuclideanGraphMotionPlanner

    // Inline Functions
    unsigned int MaskedCompleteSampledEuclideanGraphMotionPlanner::numMasked() const
    {
        return m_indices.size();
    }

    unsigned int MaskedCompleteSampledEuclideanGraphMotionPlanner::totalNumber() const
    {
        return m_sub_motion_planners.size();
    }

}  // namespace grstapse