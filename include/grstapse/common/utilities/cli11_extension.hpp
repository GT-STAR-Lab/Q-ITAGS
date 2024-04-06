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
#include <ranges>

#include <type_traits>
// External
#include <cli11/cli11.hpp>
#include <magic_enum/magic_enum.hpp>
// Local
#include "grstapse/common/utilities/magic_enum_extension.hpp"

/*!
 * \brief This file contains several functions to extend the functionality of CLI11
 *
 * \file cli11_extension.hpp
 * \see https://github.com/CLIUtils/CLI11
 */

namespace grstapse::cli11_ext
{
    /*!
     * \brief Container used to have extra command line arguments that are only available when certain flags are set
     */
    class ExtraArgument
    {
       public:
        /*!
         * \brief Constructor
         *
         * \tparam AssignTo
         * \tparam ConvertTo
         *
         * \param name
         * \param arg
         * \param description
         */
        template <typename AssignTo,
                  typename ConvertTo                                                       = AssignTo,
                  std::enable_if_t<!std::is_const<ConvertTo>::value, CLI::detail::enabler> = CLI::detail::dummy>
        ExtraArgument(const std::string& name, AssignTo& arg, const std::string& description)
            : m_name(name)
            , m_description(description)
        {
            m_callback = [&arg](const CLI::results_t& res) {  // comment for spacing
                return CLI::detail::lexical_conversion<AssignTo, ConvertTo>(res, arg);
            };
        }

        //! \returns The name of this extra argument's option
        [[nodiscard]] inline const std::string& name() const
        {
            return m_name;
        }

        //! \returns
        [[nodiscard]] inline const std::function<bool(const CLI::results_t&)>& callback() const
        {
            return m_callback;
        }

        //! \returns The description of this extra argument's option
        [[nodiscard]] inline const std::string& description() const
        {
            return m_description;
        }

       private:
        std::string m_name;
        std::string m_description;
        std::function<bool(const CLI::results_t&)> m_callback;
    };

    template <std_ext::Enum E>
    class CliEnumParametersBase
    {
       public:
        /*!
         * \brief Constructor
         *
         * \tparam E An enum type
         *
         * \param name The name of the command line argument
         * \param description The description of the command line argument
         * \param option_descriptions A map of descriptions for each of the enumerators in E
         */
        explicit CliEnumParametersBase(const std::string& name,
                                       const std::string& description,
                                       const std::map<E, std::string>& option_descriptions)
            : m_name(name)
            , m_description(description)
            , m_option_descriptions(option_descriptions)
        {
            assert(option_descriptions.size() == magic_enum::enum_count<E>());
        }

        //! \returns The name of the enum's option group
        [[nodiscard]] inline const std::string& name() const
        {
            return m_name;
        }

        //! \returns The description of the enum's option group
        [[nodiscard]] virtual inline std::string description() const = 0;

        //! A map of the individual enumerators and their respective descriptions
        [[nodiscard]] inline const std::map<E, std::string>& optionDescriptions() const
        {
            return m_option_descriptions;
        }

        /*! Adds a command line argument that is specific to one of the enumerators
         *
         * \note The argument will not be accept if a different enumerator is chosen
         */
        inline void addExtraArgument(E e, ExtraArgument& argument)
        {
            m_extra_args.emplace(e, argument);
        }

        /*! Adds a command line argument that is specific to one of the enumerators
         *
         * \note The argument will not be accept if a different enumerator is chosen
         */
        inline void addExtraArgument(E e, ExtraArgument&& argument)
        {
            m_extra_args.emplace(e, argument);
        }

        //! \returns Whether \p e has extra arguments associated with it
        [[nodiscard]] inline bool containsExtraArgs(E e) const
        {
            return m_extra_args.contains(e);
        }

        //! \returns A view to the extra argument options for \p e
        [[nodiscard]] auto extraArguments(E e) const
        {
            auto begin_end = m_extra_args.equal_range(e);
            return std::ranges::subrange(begin_end.first, begin_end.second) | std::views::values;
        }

       protected:
        std::string m_name;
        std::string m_description;
        std::map<E, std::string> m_option_descriptions;
        std::multimap<E, ExtraArgument> m_extra_args;
    };

    /*!
     * \brief A container for the information needed to setup command line arguments for a an enum
     *
     * \tparam E
     */
    template <std_ext::Enum E>
    class CliEnumParameters : public CliEnumParametersBase<E>
    {
        using Base_ = CliEnumParametersBase<E>;

       public:
        /*!
         * \brief Constructor
         *
         * \tparam E An enum type
         *
         * \param arg The variable to populate
         * \param name The name of the command line argument
         * \param description The description of the command line argument
         * \param option_descriptions A map of descriptions for each of the enumerators in E
         */
        CliEnumParameters(E& arg,
                          const std::string& name,
                          const std::string& description,
                          const std::map<E, std::string>& option_descriptions)
            : Base_(name, description, option_descriptions)
            , m_arg(arg)
        {}

        //! \returns A reference to the argument to populate
        [[nodiscard]] inline E& arg()
        {
            return m_arg;
        }

        //! \copydoc CliEnumParametersBase
        [[nodiscard]] inline std::string description() const override
        {
            std::string_view e_name = magic_enum::enum_name(m_arg);
            if(e_name.starts_with("e_"))
            {
                e_name = e_name.substr(2);
            }
            return fmt::format("{0:s} (default: {1:s})\n", Base_::m_description, e_name);
        }

       private:
        E& m_arg;
    };

    /*!
     * \brief A container for the information needed to setup command line arguments for a list of enums
     *
     * \tparam E An enum type
     */
    template <std_ext::Enum E>
    class CliEnumListParameters : public CliEnumParametersBase<E>
    {
        using Base_ = CliEnumParametersBase<E>;

       public:
        /*!
         * \brief Constructor
         *
         * \tparam E An enum type
         *
         * \param arg The variable to populate
         * \param name The name of the command line argument
         * \param description The description of the command line argument
         * \param option_descriptions A map of descriptions for each of the enumerators in E
         */
        CliEnumListParameters(std::set<E>& arg,
                              const std::string& name,
                              const std::string& description,
                              const std::map<E, std::string>& option_descriptions)
            : Base_(name, description, option_descriptions)
            , m_arg(arg)
        {}

        //! \returns A reference to the argument to populate
        [[nodiscard]] inline std::set<E>& arg()
        {
            return m_arg;
        }

        //! \copydoc CliEnumParametersBase
        [[nodiscard]] inline std::string description() const override
        {
            std::string default_description;
            for(E e: m_arg)
            {
                std::string_view e_name = magic_enum::enum_name(e);
                if(e_name.starts_with("e_"))
                {
                    e_name = e_name.substr(2);
                }
                default_description += fmt::format("{0:s}, ", e_name);
            }

            return fmt::format("{0:s} (default: {{{1:s}}})\n",
                               Base_::m_description,
                               std::string_view(default_description).substr(0, default_description.size() - 2));
        }

       private:
        std::set<E>& m_arg;
    };

    /*!
     * Custom formatter for CLI11
     *
     * \todo(Andrew): fix option group header
     * \todo(Andrew): fix mutual exclusion message
     */
    class CustomCliFormatter : public CLI::Formatter
    {
       public:
    };

    /*!
     * \brief Custom setup for adding an enum command line argument
     *
     * \tparam E An enum type
     *
     * \param params The parameters for setting up the command line arguments for this enum
     */
    template <std_ext::Enum E>
    CLI::App* createEnumArgument(CLI::App& app, CliEnumParameters<E>&& params)
    {
        CLI::App* option_group = app.add_option_group(params.name(), params.description())->require_option(0, 1);
        E& arg                 = params.arg();
        for(const auto& [enumerator, description]: params.optionDescriptions())
        {
            std::string_view e_name = magic_enum::enum_name(enumerator);
            // Ignore the "e_"
            if(e_name.starts_with("e_"))
            {
                e_name = e_name.substr(2);
            }

            CLI::Option* flag = option_group
                                    ->add_flag_callback(
                                        fmt::format("--{0:s}", e_name),
                                        [&arg, enumerator = enumerator]()
                                        {
                                            arg = enumerator;
                                        },
                                        description)
                                    ->ignore_case();

            if(params.containsExtraArgs(enumerator))
            {
                for(const ExtraArgument& argument: params.extraArguments(enumerator))
                {
                    app.add_option(argument.name(), argument.callback(), argument.description())->needs(flag);
                }
            }
        }
        return option_group;
    }

    /*!
     * \brief Custom setup for adding a list of enums command line argument
     *
     * \tparam E An enum type
     *
     * \param params The parameters for setting up the command line arguments for this list of enums
     */
    template <std_ext::Enum E>
    CLI::App* createEnumListArgument(CLI::App& app, CliEnumListParameters<E>&& params)
    {
        CLI::App* option_group = app.add_option_group(params.name(), params.description());
        std::vector<std::shared_ptr<CLI::Option_group>> suboption_groups;
        std::set<E>& arg = params.arg();

        for(const auto& [enumerator, description]: params.optionDescriptions())
        {
            std::string_view e_name = magic_enum::enum_name(enumerator);
            // Ignore the "e_"
            if(e_name.starts_with("e_"))
            {
                e_name = e_name.substr(2);
            }

            CLI::Option* flag = option_group
                                    ->add_flag_callback(
                                        fmt::format("--{0:s}", e_name),
                                        [&arg, enumerator = enumerator]()
                                        {
                                            arg.insert(enumerator);
                                        },
                                        description)
                                    ->ignore_case();

            if(params.containsExtraArgs(enumerator))
            {
                for(const ExtraArgument& argument: params.extraArguments(enumerator))
                {
                    option_group->add_option(argument.name(), argument.callback(), argument.description());
                }
            }
        }
        return option_group;
    }

}  // namespace grstapse::cli11_ext