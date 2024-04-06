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

#ifndef BFG_DUCK_DISPATCH_H
#define BFG_DUCK_DISPATCH_H

#include <type_traits>
#include <utility>

namespace bfg
{
    namespace tag_invoke_detail
    {
        struct tag_invoke_t
        {
            template <typename Tag, typename... Args>
            constexpr auto operator()(Tag tag, Args &&...args) const
                noexcept(noexcept(tag_invoke(static_cast<Tag &&>(tag), static_cast<Args &&>(args)...)))
                    -> decltype(tag_invoke(static_cast<Tag &&>(tag), static_cast<Args &&>(args)...))
            {
                return tag_invoke(static_cast<Tag &&>(tag), static_cast<Args &&>(args)...);
            }
        };

        template <class Tag>
        struct tag_invoke_value_t
        {
            static constexpr Tag value{};
        };

        template <class Tag>
        constexpr Tag tag_invoke_value_t<Tag>::value;

        static constexpr auto &tag_invoke = tag_invoke_value_t<tag_invoke_t>::value;

        template <typename Tag, typename... Args>
        auto tag_invoke_check(int) noexcept(noexcept(tag_invoke(::std::declval<Tag &&>(),
                                                                ::std::declval<Args &&>()...)))
            -> decltype(static_cast<void>(tag_invoke(::std::declval<Tag &&>(), ::std::declval<Args &&>()...)),
                        std::true_type{});

        template <typename Tag, typename... Args>
        std::false_type tag_invoke_check(...) noexcept(false);
    }  // namespace tag_invoke_detail

    /* tag::reference[]
    [#bfg_tag_invoke]
    = `bfg::tag_invoke`
    The singleton function object that dispatches, through ADL, to the overload set
    of the specific customization point object tag by calling the unqualified
    `tag_invoke` function.
    */ // end::reference[]
    using tag_invoke_detail::tag_invoke;

    /* tag::reference[]
    [#bfg_tag_invoke_result_t]
    = `bfg::tag_invoke_result_t`
    Obtains, for a customization point object type of `Tag` called wih the arguments
    `Args...`, the result type of calling that CPO.
    */ // end::reference[]
    template <typename Tag, typename... Args>
    using tag_invoke_result_t = decltype(tag_invoke(::std::declval<Tag>(), ::std::declval<Args>()...));

    /* tag::reference[]
    [#bfg_tag_invoke_is_nothrow]
    = `bfg::tag_invoke_is_nothrow`
    Detects if the call to the given customization point object, of type `Tag` and
    the argument types of `Args...`, is marked `noexcept(true)`.
    */ // end::reference[]
    template <typename Tag, typename... Args>
    struct tag_invoke_is_nothrow
    {
        static constexpr bool value =
            std::integral_constant<bool, noexcept(tag_invoke_detail::tag_invoke_check<Tag, Args...>(0))>::value;
    };

    /* tag::reference[]
    [#bfg_tag]
    = `bfg::tag`
    The `bfg::tag` template defines the customization point object type. To use
    inherit from it with CRTP. For example:
    [source,cpp]
    ----
    struct run_t final : bfg::tag<run_t> {};
    ----
    This only defines the type for the CPO. To define the CPO itself you need to use
    the `tag_invoke_v` utility.
    */ // end::reference[]
    template <typename Tag>
    struct tag
    {
        template <typename... Args>
        constexpr auto operator()(Args &&...args) const
            noexcept(::bfg::tag_invoke_is_nothrow<Tag, decltype(args)...>::value)
                -> ::bfg::tag_invoke_result_t<Tag, decltype(args)...>
        {
            return ::bfg::tag_invoke(*static_cast<const Tag *>(this), ::std::forward<Args>(args)...);
        }
    };

    /* tag::reference[]
    [#bfg_tag_invoke_v]
    = `bfg::tag_invoke_v`
    The `tag_invoke_v` function obtains a reference to the customization point
    object singleton.
    */ // end::reference[]
    template <typename T>
    constexpr const T &tag_invoke_v(T &&)
    {
        return tag_invoke_detail::tag_invoke_value_t<T>::value;
    }

/* tag::reference[]
[#bfg_tag_invoke_def]
= `BFG_TAG_INVOKE_DEF`
The `BFG_TAG_INVOKE_DEF(Tag)` macro defines the customization point object with
a corresponding type. The argument is the name of the CPO to define the type
of the generated CPO will be the `Tag` post-fixed with `_t`. For a CPO named
`run`, i.e. `BFG_TAG_INVOKE_DEV(run);` it produces:
[source,cpp]
----
static struct run_t final : ::bfg::tag<run_t> {}
  const & run = ::bfg::tag_invoke_v(run_t {});
----
*/ // end::reference[]
#define BFG_TAG_INVOKE_DEF(Tag)                                                                                        \
    static struct Tag##_t final : ::bfg::tag<Tag##_t>                                                                  \
    {                                                                                                                  \
    } const &Tag = ::bfg::tag_invoke_v(Tag##_t{})

}  // namespace bfg

#endif