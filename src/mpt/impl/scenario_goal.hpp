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
#ifndef MPT_IMPL_SCENARIO_GOAL_HPP
#define MPT_IMPL_SCENARIO_GOAL_HPP

#include <type_traits>
#include "scenario_state.hpp"
#include "always_false.hpp"

namespace unc::robotics::mpt::impl {

    template <class T, class = void>
    struct scenario_has_goal_class : std::false_type {};

    template <class T>
    struct scenario_has_goal_class<T, std::void_t<decltype( std::declval<T>().goal() )>> : std::true_type {};

    template <class T, class = void>
    struct scenario_has_goal_method : std::false_type {};

    template <class T>
    struct scenario_has_goal_method<T, std::void_t<decltype( std::declval<T>().isGoal( std::declval<scenario_state_t<T>>() ))>>
        : std::true_type {};

    template <class T, bool classBased, bool methodBased>
    struct scenario_goal_selector {
        static_assert(always_false<T>, "scenario must have a goal() or goal(State) method, but not both");
    };

    // Template specialiation for scenarios that have a no-argument
    // goal method that returns a goal object.
    template <class T>
    struct scenario_goal_selector<T, true, false> {
        // using type = std::decay_t<decltype( std::declval<T>().goal() )>;

        static decltype(auto) check(const T& scenario, const scenario_state_t<T>& q) {
            return scenario.goal()(scenario.space(), q);
        }
    };

    template <class T>
    struct scenario_goal_selector<T, false, true> {
        static decltype(auto) check(const T& scenario, const scenario_state_t<T>& q) {
            return scenario.isGoal(q);
        }
    };
    
    template <class T>
    struct scenario_goal : scenario_goal_selector<
        T,
        scenario_has_goal_class<T>::value,
        scenario_has_goal_method<T>::value>
    {
    };
}

#endif
