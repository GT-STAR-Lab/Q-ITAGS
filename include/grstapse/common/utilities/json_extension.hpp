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
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <vector>
// External
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"

/*!
 * \brief Several utility function to extend the functionality of the nlohmann/json library. This also includes several
 *       functions for converting 3rd party data structures to/from json.
 *
 * \note from_json/to_json functions for data structures local to this project should not be included in this file.
 *       Those functions should be in their respective headers and sources.
 *
 * \file json_extension.hpp
 */

namespace nlohmann
{
    //! Concept version of nlohmann::detail::has_from_json
    template <typename T>
    concept has_from_json_concept = nlohmann::detail::has_from_json<nlohmann::json, T>::value;
    //! Concept version of nlohmann::detail::has_non_default_from_json
    template <typename T>
    concept has_non_default_from_json_concept = nlohmann::detail::has_non_default_from_json<nlohmann::json, T>::value;
    //! Concept version of nlohmann::detail::has_to_json
    template <typename T>
    concept has_to_json_concept = nlohmann::detail::has_to_json<nlohmann::json, T>::value;

    /*!
     * \brief Serialization to/Deserialization from json for unique_ptr's
     *
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <typename T>
    requires(has_from_json_concept<T> || has_to_json_concept<T>) struct adl_serializer<std::unique_ptr<T>>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param p The unique_ptr to populate
         */
        static void from_json(const json& j, std::unique_ptr<T>& p) requires has_from_json_concept<T>
        {
            if(j.is_null())
            {
                p = nullptr;
            }
            else
            {
                p = std::make_unique<T>();
                j.get_to<T>(*p);
            }
        }

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param p The unique_ptr to serialize
         */
        static void to_json(json& j, const std::unique_ptr<T>& p) requires has_to_json_concept<T>
        {
            if(p)
            {
                j = *p;
            }
            else
            {
                j = nullptr;
            }
        }
    };

    /*!
     * Serialization to/Deserialization from json for shared_ptr's when the base type has a default constructor
     *
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <typename T>
    requires(has_from_json_concept<T> || has_to_json_concept<T>) struct adl_serializer<std::shared_ptr<T>>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param p The shared_ptr to populate
         */
        static void from_json(const json& j, std::shared_ptr<T>& p) requires has_from_json_concept<T>
        {
            if(j.is_null())
            {
                p = nullptr;
            }
            else
            {
                p = std::make_shared<T>();
                j.get_to<T>(*p);
            }
        }

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param p The shared_ptr to serialize
         */
        static void to_json(json& j, const std::shared_ptr<T>& p) requires has_to_json_concept<T>
        {
            if(p)
            {
                j = *p;
            }
            else
            {
                j = nullptr;
            }
        }
    };

    /*!
     * \brief Serialization to/Deserialization from json for optionals
     *
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <typename T>
    requires(has_from_json_concept<T> || has_non_default_from_json_concept<T> ||
             has_to_json_concept<T>) struct adl_serializer<std::optional<T>>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param opt The optional to populate
         */
        static void from_json(const json& j, std::optional<T>& opt) requires has_from_json_concept<T>
        {
            if(j.is_null())
            {
                opt = std::nullopt;
            }
            else
            {
                j.get_to<T>(*opt);
            }
        }

        /*!
         * \brief Move only deserialize from json
         *
         * \param j The json to deserialize from
         * \param opt The optional to populate
         */
        static std::optional<T> from_json(const json& j) requires has_non_default_from_json_concept<T>
        {
            if(j.is_null())
            {
                return std::nullopt;
            }
            else
            {
                return j.get<T>();
            }
        }

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param opt The optional to serialize
         */
        static void to_json(json& j, const std::optional<T>& opt) requires has_to_json_concept<T>
        {
            if(opt)
            {
                j = *opt;
            }
            else
            {
                j = nullptr;
            }
        }
    };

    /*!
     * \brief Serialization to/Deserialization from json for matrices and vectors
     *
     * \see Eigen::MatrixBase
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <typename Derived>
    struct adl_serializer<Eigen::MatrixBase<Derived>>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param m The matrix/vector to populate
         */
        static void from_json(const json& j, Eigen::MatrixBase<Derived>& m)
        {
            using Scalar = typename Eigen::MatrixBase<Derived>::Scalar;

            for(std::size_t row = 0, num_rows = j.size(); row < num_rows; ++row)
            {
                const auto& json_row = j.at(row);
                for(std::size_t col = 0, num_cols = json_row.size(); col < num_cols; ++col)
                {
                    const auto& value = json_row.at(col);
                    m(row, col)       = value.get<Scalar>();
                }
            }
        }

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param m The matrix/vector to serialize
         */
        static void to_json(json& j, const Eigen::MatrixBase<Derived>& m)
        {
            for(unsigned int row = 0, num_rows = m.rows(); row < num_rows; ++row)
            {
                nlohmann::json column = nlohmann::json::array();
                for(unsigned int col = 0, num_cols = m.cols(); col < num_cols; ++col)
                {
                    column.push_back(m(row, col));
                }
                j.push_back(column);
            }
        }
    };

    /*!
     * \brief Serialization to/Deserialization from json for matrices and vectors
     *
     * \see Eigen::Matrix
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    struct adl_serializer<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param m The matrix/vector to populate
         */
        static void from_json(const json& j, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
        {
            const unsigned int num_rows = m.rows() > 0 ? m.rows() : j.size();
            if(num_rows == 0)
            {
                return;
            }

            if(j[0].is_array())
            {
                const unsigned int num_cols = m.cols() > 0 ? m.cols() : j[0].size();
                m.resize(num_rows, num_cols);

                for(std::size_t row = 0; row < num_rows; ++row)
                {
                    const json& json_row = j.at(row);
                    for(std::size_t col = 0; col < num_cols; ++col)
                    {
                        const json& value = json_row.at(col);
                        m(row, col)       = value.get<_Scalar>();
                    }
                }
            }
            else if(j[0].is_number())
            {
                m.resize(num_rows, 1);
                for(std::size_t row = 0; row < num_rows; ++row)
                {
                    m(row, 0) = j.at(row).get<_Scalar>();
                }
            }
            else
            {
                throw grstapse::createLogicError("malformed json for eigen vector or matrix");
            }
        }

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param m The matrix/vector to serialize
         */
        static void to_json(json& j, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
        {
            const unsigned int num_rows = m.rows();
            const unsigned int num_cols = m.cols();

            j = nlohmann::json::array();
            if(num_cols > 1 && num_rows > 1)
            {
                for(unsigned int row = 0; row < num_rows; ++row)
                {
                    nlohmann::json column = nlohmann::json::array();
                    for(unsigned int col = 0; col < num_cols; ++col)
                    {
                        column.push_back(m(row, col));
                    }
                    j.push_back(column);
                }
            }
            else if(num_cols == 1)
            {
                for(unsigned int row = 0; row < num_rows; ++row)
                {
                    j.push_back(m(row, 0));
                }
            }
            else if(num_rows == 1)
            {
                for(unsigned int col = 0; col < num_cols; ++col)
                {
                    j.push_back(m(0, col));
                }
            }
            else
            {
                throw grstapse::createLogicError("Don't know how we got here...");
            }
        }
    };

    /*!
     * \brief Serialization to/Deserialization from json for quaternions
     *
     * \see Eigen::QuaternionBase
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <typename Derived>
    struct adl_serializer<Eigen::QuaternionBase<Derived>>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param q The quaternion to populate
         */
        static void from_json(const json& j, Eigen::QuaternionBase<Derived>& q)
        {
            j[grstapse::constants::k_qw] = q.w();
            j[grstapse::constants::k_qx] = q.x();
            j[grstapse::constants::k_qy] = q.y();
            j[grstapse::constants::k_qz] = q.z();
        }

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param q The quaternion to populate
         */
        static void to_json(json& j, const Eigen::QuaternionBase<Derived>& q)
        {
            using Scalar = typename Eigen::QuaternionBase<Derived>::Scalar;
            j.at(grstapse::constants::k_qw).get_to<Scalar>(q.w());
            j.at(grstapse::constants::k_qx).get_to<Scalar>(q.x());
            j.at(grstapse::constants::k_qy).get_to<Scalar>(q.y());
            j.at(grstapse::constants::k_qz).get_to<Scalar>(q.z());
        }
    };

    /*!
     * \brief Serialization to/Deserialization from json for real numbered vector bounds
     *
     * \see ompl::base::RealVectorBounds
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::base::RealVectorBounds>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         *
         * \returns A real numbered vector bounds
         */
        static ompl::base::RealVectorBounds from_json(const json& j);

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param b A real numbered vector bounds to serialize
         */
        static void to_json(json& j, const ompl::base::RealVectorBounds& b);
    };

    /*!
     * \brief Serialization to/Deserialization from json for states in a SE2 state space
     *
     * \see ompl::base::SE2StateSpace::StateType
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::base::SE2StateSpace::StateType>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param s A state from an SE2 state space to populate
         */
        static void from_json(const json& j, ompl::base::SE2StateSpace::StateType& s);

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param s A state from an SE2 state space to serialize
         */
        static void to_json(json& j, const ompl::base::SE2StateSpace::StateType& s);
    };

    /*!
     * \brief Serialization to/Deserialization from json for a SE2 state space
     *
     * \see ompl::base::SE2StateSpace
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::base::SE2StateSpace>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param s A SE2 state space to populate
         */
        static void from_json(const json& j, ompl::base::SE2StateSpace& s);

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param s A SE2 state space to serialize
         */
        static void to_json(json& j, const ompl::base::SE2StateSpace& s);
    };

    /*!
     * \brief Serialization to/Deserialization from json for states in a SE3 state space
     *
     * \see ompl::base::SE3StateSpace::StateType
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::base::SE3StateSpace::StateType>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param s A state from the SE3 state space to populate
         */
        static void from_json(const json& j, ompl::base::SE3StateSpace::StateType& s);

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param s A state from the SE3 state space to serialize
         */
        static void to_json(json& j, const ompl::base::SE3StateSpace::StateType& s);
    };

    /*!
     * \brief Serialization to/Deserialization from json for a SE3 state space
     *
     * \see ompl::base::SE3StateSpace
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::base::SE3StateSpace>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param s A SE3 state space to populate
         */
        static void from_json(const json& j, ompl::base::SE3StateSpace& s);

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param s A SE3 state space to serialize
         */
        static void to_json(json& j, const ompl::base::SE3StateSpace& s);
    };

    /*!
     * \brief Serialization to/Deserialization from json for states in a SO2 state space
     *
     * \see ompl::base::SO3StateSpace::StateType
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::base::SO3StateSpace::StateType>
    {
        /*!
         * \brief Deserialize from json
         *
         * \param j The json to deserialize from
         * \param s A state from the SO3 state space to populate
         */
        static void from_json(const json& j, ompl::base::SO3StateSpace::StateType& s);

        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param s A state from the SO3 state space to serialize
         */
        static void to_json(json& j, const ompl::base::SO3StateSpace::StateType& s);
    };

    /*!
     * \brief Serialization to json for an OMPL PathGeometric
     *
     * \see ompl::geometric::PathGeometric
     * \see https://json.nlohmann.me/features/arbitrary_types/
     */
    template <>
    struct adl_serializer<ompl::geometric::PathGeometric>
    {
        /*!
         * \brief Serialize to json
         *
         * \param j The json to serialize to
         * \param p A geometric path to serialize
         */
        static void to_json(json& j, const ompl::geometric::PathGeometric& p);
    };
}  // namespace nlohmann

namespace grstapse::json_ext
{
    template <typename T>
    T loadJsonFromFile(const std::string& filename)
    {
        if(!std::filesystem::exists(filename))
        {
            throw createLogicError(fmt::format("File '{0:s}' doesn't exists", filename));
        }
        std::ifstream fin(filename);
        if(!fin.is_open())
        {
            throw createLogicError(fmt::format("Error opening file '{0:s}'", filename));
        }
        nlohmann::json j;
        fin >> j;
        return j.get<T>();
    }

    /*!
     * \brief Validates json field types
     *
     * \param j The json
     * \param required The required fields
     * \param optionals The optional fields
     */
    void validateJson(
        const nlohmann::json& j,
        const std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>>& required,
        const std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>>& optionals = {},
        const std::experimental::source_location location = std::experimental::source_location::current());

    /*!
     * \brief Validates json field types
     *
     * \param j The json
     * \param required The required fields
     * \param optionals The optional fields
     */
    void validateJson(
        const nlohmann::json& j,
        const std::vector<std::pair<const char* const, nlohmann::json::value_t>>& required,
        const std::vector<std::pair<const char* const, nlohmann::json::value_t>>& optionals = {},
        const std::experimental::source_location location = std::experimental::source_location::current());
}  // namespace grstapse::json_ext