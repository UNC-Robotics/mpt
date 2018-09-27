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
#ifndef MPT_DISCRETE_MOTION_VALIDATOR_HPP
#define MPT_DISCRETE_MOTION_VALIDATOR_HPP

#include "log.hpp"
#include <array>

namespace unc::robotics::mpt {

    template <
        typename Space_,
        typename StateValidator>
    class DiscreteMotionValidator : StateValidator {
        using Space = Space_;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;

        // should be a power of 2 for efficiency
        static constexpr std::size_t fixedBisectQueueSize_ = 256;

        Space space_;
        Distance invStepSize_;

    public:
        template <typename ... Args>
        DiscreteMotionValidator(const Space& space, Distance stepSize, Args&& ... args)
            : StateValidator(std::forward<Args>(args)...)
            , space_(space)
            , invStepSize_(1/stepSize)
        {
            MPT_LOG(DEBUG) << "stepSize = " << stepSize << ", inv = " << invStepSize_;
        }

        using StateValidator::operator();

        bool operator() (const State& from, const State& to) const {
            // Assume the caller has verified that from is valid.
            assert(StateValidator::operator()(from));

            if (!static_cast<const StateValidator&>(*this)(to))
                return false;

            std::size_t steps = std::ceil(space_.distance(from, to) * invStepSize_);
            if (steps < 2)
                return true;

            Distance delta = Distance(1) / Distance(steps);

            // MPT_LOG(DEBUG) << "steps = " << steps << ", delta = " << delta;
#if 0
            // link: 28936 calls, 550.036 us avg, 3.819 us min, 23091.8 us max, 1.59158e+07 us total
            for (std::size_t i=1 ; i<steps ; ++i)
                if (!static_cast<const StateValidator&>(*this)(space_.interpolate(from, to, i * delta)))
                    return false;
#else
            // link: 35978 calls, 440.951 us avg, 4.672 us min, 23178.1 us max, 1.58645e+07 us total

            // we could use a std::deque here instead but it requires
            // an allocation.  we'd prefer to avoid that since this
            // method is called often.  So instead, we use a fixed
            // size queue until it reaches capacity.
            //
            // Another option to try might be a std::vector with a
            // reserved size of 'steps/2 + 1' (since that will be the
            // maximum queue size, and at least will only require 1
            // allocation).
            std::array<std::pair<std::size_t, std::size_t>, fixedBisectQueueSize_> queue;
            queue[0] = std::make_pair(std::size_t(1), steps-1);
            std::size_t qStart = 0, qEnd = 1;

            while (qStart != qEnd) {
                auto [min, max] = queue[qStart++ % fixedBisectQueueSize_];
                if (min == max) {
                    if (!static_cast<const StateValidator&>(*this)(interpolate(space_, from, to, min * delta)))
                        return false;
                } else if (qEnd + 2 < qStart + fixedBisectQueueSize_) {
                    std::size_t mid = (min + max) / 2;
                    if (!static_cast<const StateValidator&>(*this)(interpolate(space_, from, to, mid * delta)))
                        return false;
                    if (min < mid)
                        queue[qEnd++ % fixedBisectQueueSize_] = std::make_pair(min, mid-1);
                    if (mid < max)
                        queue[qEnd++ % fixedBisectQueueSize_] = std::make_pair(mid+1, max);
                } else {
                    // buffer is full, and if the checks pass, we will
                    // need to insert into the queue.  here we switch
                    // to checking sequentially
                    for (std::size_t i=min ; i<=max ; ++i)
                        if (!static_cast<const StateValidator&>(*this)(interpolate(space_, from, to, i * delta)))
                            return false;
                }
            }
#endif
            return true;
        }
    };

}

#endif // MPT_DISCRETE_MOTION_VALIDATOR_HPP
