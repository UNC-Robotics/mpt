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
#ifndef MPT_IMPL_SCENARIO_GOAL_SAMPLER_HPP
#define MPT_IMPL_SCENARIO_GOAL_SAMPLER_HPP

#include <type_traits>
#include "scenario_state.hpp"
#include "scenario_goal.hpp"
#include "always_false.hpp"

namespace unc::robotics::mpt::impl {

    template <class T, class RNG, bool, bool>
    struct scenario_has_goal_sampler_selector : std::false_type {};

    template <class T, class RNG>
    struct scenario_has_goal_sampler_selector<T, RNG, true, false>
        : goal_has_sampler<std::decay_t<decltype( std::declval<T>().goal() )>>
    {
    };

    template <class T, class RNG, class = void>
    struct scenario_has_goal_sampler_method : std::false_type {};

    template <class T, class RNG>
    struct scenario_has_goal_sampler_method<
        T, RNG,
        std::void_t<decltype( std::declval<T>().sampleGoal( std::declval<RNG&>() ))>>
        : std::true_type {};

    template <class T, class RNG>
    struct scenario_has_goal_sampler_selector<T, RNG, false, true>
        : scenario_has_goal_sampler_method<T, RNG>
    {
    };
    
    template <class T, class RNG>
    static constexpr bool scenario_has_goal_sampler_v = scenario_has_goal_sampler_selector<
        T, RNG,
        scenario_has_goal_class<T>::value,
        scenario_has_goal_method<T>::value>::value;


    template <class T, class RNG, bool, bool>
    struct scenario_goal_sampler_selector;

    template <class T, class RNG>
    struct scenario_goal_sampler_selector<T, RNG, true, false>
        : GoalSampler<std::decay_t<decltype( std::declval<T>().goal() )>>
    {
        scenario_goal_sampler_selector(const T& scenario)
            : GoalSampler<std::decay_t<decltype( std::declval<T>().goal() )>>(
                scenario.goal())
        {
        }
    };

    template <class T, class RNG>
    struct scenario_goal_sampler_selector<T, RNG, false, true> {
        const T& scenario_;

        scenario_goal_sampler_selector(const T& scenario)
            : scenario_(scenario)
        {
        }

        decltype(auto) operator() (RNG& rng) {
            return scenario_.sampleGoal(rng);
        }
    };
    
    template <class T, class RNG>
    using scenario_goal_sampler_t = scenario_goal_sampler_selector<
        T, RNG,
        scenario_has_goal_class<T>::value,
        scenario_has_goal_method<T>::value>;
        
}

#endif
