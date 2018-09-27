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
#ifndef MPT_UNIFORM_BOX_SAMPLER_HPP
#define MPT_UNIFORM_BOX_SAMPLER_HPP

#include <random>
#include <Eigen/Dense>
#include <cassert>

namespace unc::robotics::mpt {
    template <typename Space>
    class UniformBoxSampler {
        using Scalar = typename Space::Distance;
        using Type = typename Space::Type;
        static constexpr int dimensions = Space::kDimensions;

        BoxBounds<Scalar, dimensions> bounds_;

    public:
        template <typename ... Args>
        UniformBoxSampler(Args&& ... args)
            : bounds_(std::forward<Args>(args)...)
        {
        }

        template <typename RNG>
        Type operator() (RNG& rng) const {
            Type q;
            for (unsigned i=0 ; i < bounds_.size() ; ++i) {
                std::uniform_real_distribution<Scalar> dist(bounds_.min()[i], bounds_.max()[i]);
                Space::coeff(q, i) = dist(rng);
            }
            return q;
        }
    };
}

#endif
