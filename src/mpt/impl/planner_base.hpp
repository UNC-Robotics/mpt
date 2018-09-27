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
#ifndef MPT_IMPL_PLANNER_BASE_HPP
#define MPT_IMPL_PLANNER_BASE_HPP

#include <chrono>
#include "condition_timer.hpp"

namespace unc::robotics::mpt::impl {
    template <typename Derived>
    class PlannerBase {
    public:
        // The time based solve methods use ConditionTimer instead of
        // SleepTimer, since it is more well-behaved in the case when
        // solve() terminates for a reason other than the doneFn
        // predicate returning true (e.g., an exception)

        // TODO: when solve(fn) is only going to call the done
        // predicate a few times, it may be better to just call
        // Clock::now() directly in the done predicate, instead of
        // incurring the overhead of starting another thread just for
        // the conditioned timed wait.

        template <typename Rep, typename Period>
        void solveFor(const std::chrono::duration<Rep, Period>& duration) {
            ConditionTimer timer(duration);
            static_cast<Derived*>(this)->solve(timer.doneFn());
        }

        template <class Clock, class Duration>
        void solveUntil(const std::chrono::time_point<Clock, Duration>& endTime) {
            ConditionTimer timer(endTime);
            static_cast<Derived*>(this)->solve(timer.doneFn());
        }

        template <typename DoneFn, typename Rep, typename Period>
        void solveFor(DoneFn doneFn, const std::chrono::duration<Rep, Period>& duration) {
            ConditionTimer timer(duration);
            static_cast<Derived*>(this)->solve(timer.doneFn(doneFn));
        }

        template <typename DoneFn, class Clock, class Duration>
        void solveUntil(DoneFn doneFn, const std::chrono::time_point<Clock, Duration>& endTime) {
            ConditionTimer timer(endTime);
            static_cast<Derived*>(this)->solve(timer.doneFn(doneFn));
        }
    };
}

#endif
