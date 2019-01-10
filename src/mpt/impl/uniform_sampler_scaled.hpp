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
#ifndef MPT_IMPL_UNIFORM_SAMPLER_SCALED_HPP
#define MPT_IMPL_UNIFORM_SAMPLER_SCALED_HPP

#include <random>
#include <cmath>
#include "constants.hpp"
#include "../scaled_space.hpp"

namespace unc::robotics::mpt {
    template <typename T, typename M, typename W, typename Bounds>
    struct UniformSampler<Space<T, Scaled<M, W>>, Bounds>
        : UniformSampler<Space<T, M>, Bounds>
    {
        using Base = UniformSampler<Space<T, M>, Bounds>;

        UniformSampler(const Space<T, Scaled<M, W>>& space, const Bounds& bounds)
            : Base(space.space(), bounds)
        {
        }
    };

    template <class T, class M, class W, class Bounds>
    auto measure(
        const UniformSampler<Space<T, Scaled<M, W>>, Bounds>& sampler,
        const Space<T, Scaled<M, W>>& space)
    {
        return measure(
            static_cast<const UniformSampler<Space<T, M>, Bounds>&>(sampler),
            space.space()) * space.weight();
    }
}

#endif
