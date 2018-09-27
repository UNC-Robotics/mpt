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
#ifndef MPT_GOAL_STATE_HPP
#define MPT_GOAL_STATE_HPP

#include "goal_sampler.hpp"
#include <utility>

namespace unc::robotics::mpt {
    template <typename Space>
    class GoalState {
        using State = typename Space::Type;
        using Distance = typename Space::Distance;

        State goal_;
        Distance radius_;

    public:
        template <typename ... Args>
        GoalState(Distance radius, Args&& ... args)
            : goal_(std::forward<Args>(args)...)
            , radius_(radius)
        {
        }

        const State& state() const {
            return goal_;
        }

        std::pair<bool, Distance> operator() (const Space& space, const State& q) const {
            Distance d = space.distance(q, goal_);
            return (d <= radius_)
                ? std::make_pair(true, Distance(0))
                : std::make_pair(false, d - radius_);
        }
    };

    template <typename Space>
    class GoalSampler<GoalState<Space>> {
        const GoalState<Space>& goal_;

    public:
        using Type = typename Space::Type;

        GoalSampler(const GoalState<Space>& goal)
            : goal_(goal)
        {
        }

        template <typename RNG>
        Type operator() (RNG&) const {
            // TODO: sample within radius
            return goal_.state();
        }
    };
}

#endif
