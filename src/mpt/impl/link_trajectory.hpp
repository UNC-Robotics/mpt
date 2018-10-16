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
#ifndef MPT_IMPL_LINK_TRAJECTORY_HPP
#define MPT_IMPL_LINK_TRAJECTORY_HPP

#include "is_invocable.hpp"
#include <optional>
#include <type_traits>
#include <variant>

namespace unc::robotics::mpt::impl {

    // link_trajectory converts the result of link() call to stored
    // trajectory type.
    template <class T>
    struct link_trajectory { using type = T; };
    template <class T>
    struct link_trajectory<std::optional<T>> { using type = T; };
    template <>
    struct link_trajectory<bool> { using type = std::monostate; };

    template <class T>
    using link_trajectory_t = typename link_trajectory<T>::type;

    // linkTrajectory converts the link result to an r-value of the
    // trajectory.  This is mostly equivalent to std::move(arg),
    // except it handles the variants oftypes that link() may return.
    template <typename T>
    std::remove_reference_t<T>&& linkTrajectory(T&& t) noexcept {
        return std::move(t);
    }
    
    template <typename T>
    std::remove_reference_t<T>&& linkTrajectory(std::optional<T>& t) noexcept {
        return std::move(t).value();
    }

    std::monostate linkTrajectory(bool) noexcept {
        return {};
    }

    template <class Fn, class State, class Traj>
    struct is_trajectory_callback
        : std::is_invocable<Fn, const State&, const Traj&, const State&, bool>
    {
    };

    template <class Fn, class State, class Traj>
    constexpr bool is_trajectory_callback_v =
        is_trajectory_callback<Fn, State, Traj>::value;

    template <class Fn, class State, class Traj>
    struct is_trajectory_reference_callback : std::false_type {};

    template <class Fn, class State, class Traj>
    struct is_trajectory_reference_callback<Fn, State, std::shared_ptr<Traj>>
        : is_trajectory_callback<Fn, State, Traj>
    {
    };

    template <class Fn, class State, class Traj>
    constexpr bool is_trajectory_reference_callback_v =
        is_trajectory_reference_callback<Fn, State, Traj>::value;


    template <class Fn, class State, class Traj>
    struct is_waypoint_callback
        : std::bool_constant<
            !is_trajectory_callback<Fn, State, Traj>::value &&
            !is_trajectory_reference_callback<Fn, State, Traj>::value &&
             std::is_invocable_v<Fn, const State&> >
    {
    };

    template <class Fn, class State, class Traj>
    constexpr bool is_waypoint_callback_v = is_waypoint_callback<Fn, State, Traj>::value;
}

#endif
