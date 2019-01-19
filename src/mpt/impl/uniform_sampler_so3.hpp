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
#ifndef MPT_IMPL_UNIFORM_SAMPLER_SO3_HPP
#define MPT_IMPL_UNIFORM_SAMPLER_SO3_HPP

#include <random>
#include <cmath>
#include "constants.hpp"
#include "../unbounded.hpp"
#include "../so3_space.hpp"

namespace unc::robotics::mpt::impl {
    template <typename T>
    struct SO3UniformSampler {
        using Space = nigh::metric::Space<T, SO3>;
        using Distance = typename Space::Distance;
        
        SO3UniformSampler(const Space&, Unbounded = Unbounded{}) {
        }

        template <typename RNG>
        T operator() (RNG& rng) const {
            std::uniform_real_distribution<Distance> dist01(0, 1);
            std::uniform_real_distribution<Distance> dist2pi(0, 2*PI<Distance>);
            Distance a = dist01(rng);
            Distance b = dist2pi(rng);
            Distance c = dist2pi(rng);

            return T(
                std::sqrt(1-a)*std::sin(b),
                std::sqrt(1-a)*std::cos(b),
                std::sqrt(a)*std::sin(c),
                std::sqrt(a)*std::cos(c));
        }
    };
}

namespace unc::robotics::mpt {
    template <typename T>
    struct UniformSampler<Space<T, SO3>, Unbounded> : impl::SO3UniformSampler<T> {
        using impl::SO3UniformSampler<T>::SO3UniformSampler;
    };

    template <typename T>
    auto measure(const UniformSampler<Space<T, SO3>, Unbounded>&, const Space<T, SO3>&) {
        using Distance = typename Space<T,SO3>::Distance;
        // half of the surface area of a unit 3-sphere.
        return impl::PI<Distance> * impl::PI<Distance>;
    }
}


#endif
