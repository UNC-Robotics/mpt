// Software License Agreement (BSD-3-Clause)
//
// Copyright 2018 The University of North Carolina at Chapel Hill
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Jeff Ichnowski

#pragma once
#ifndef MPT_IMPL_SCENARIO_LINK_HPP
#define MPT_IMPL_SCENARIO_LINK_HPP

#include "scenario_state.hpp"
#include "always_false.hpp"
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

namespace unc::robotics::mpt::impl {
    template <class T>
    struct is_valid_link : std::false_type {};
    
    // scenarios may have link method that returns any of the
    // following types:
    //
    // bool
    //
    //   True indicates the motion is valid (unobstructed), false
    //   indicates that the motion is not.
    template <>
    struct is_valid_link<bool> : std::true_type {};

    // std::optional<T>
    // std::optional<std::pair<T, State>>
    // std::optional<std::tuple<T, State>>
    //
    //   If the optional contains a value, that indicates that the
    //   embedded trajectory is a valid motion from the first argument
    //   to the second.  If 'T' is the same type as the state, then
    //   the planner will adjust to the steered value.  Otherwise the
    //   planner will store the value in the graph edge with type 'T'.
    //   When the value is not present, the motion is invalid.
    //
    //   When the optional value is a pair or tuple, the `T` value
    //   will be store in the edge.  The `State` value indicates that
    //   steering lead to a different value than the second argument
    //   of link.
    template <typename T>
    struct is_valid_link<std::optional<T>> : std::true_type {};
    
    // T*, std::shared_ptr<T>, std::unique_ptr<T>, std::weak_ptr<T>
    //
    //   Similar to std::optional<T>, except that a pointer (or
    //   shared_ptr, unique_ptr, or weak_ptr) to 'T' is stored in the
    //   edge.  Valid edges are not-null.  For the unique_ptr case,
    //   the graph (edge) takes ownership.
    //
    //   All of these return types assume that the link target is
    //   exactly reachable.  Otherwise the optional pair/tuple method
    //   should be used.
    template <typename T>
    struct is_valid_link<T*> : std::true_type {};
    template <typename T>
    struct is_valid_link<std::shared_ptr<T>> : std::true_type {};
    template <typename T, typename Deleter>
    struct is_valid_link<std::unique_ptr<T, Deleter>> : std::true_type {};
    template <typename T>
    struct is_valid_link<std::weak_ptr<T>> : std::true_type {};

    template <typename T>
    static constexpr bool is_valid_link_v = is_valid_link<T>::value;


    template <typename T>
    T&& derefLink(std::optional<T>&& link) {
        return std::move(link).get();
    }

    template <typename T>
    decltype(auto) derefLink(T&& link) {
        return std::forward<T>(link);
    }
    
    template <class T, class = void>
    struct scenario_link {
        static_assert(
            always_false<T>,
            "scenario must have a link(State,State) method");
    };

    template <class T>
    struct scenario_link<T, std::void_t<decltype(
        std::declval<const T>().link(
            std::declval<const scenario_state_t<T>&>(),
            std::declval<const scenario_state_t<T>&>() ) )> >
    {
        using type = std::decay_t<decltype(
            std::declval<T>().link(
                std::declval<const scenario_state_t<T>&>(),
                std::declval<const scenario_state_t<T>&>() )
            )>;

        static_assert(
            is_valid_link_v<type>,
            "link result type is invalid.  Expected bool, std::optional, or a pointer type.");
    };

    template <class T>
    using scenario_link_t = typename scenario_link<T>::type;
}

#endif
