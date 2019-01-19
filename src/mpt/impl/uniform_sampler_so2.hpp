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
#ifndef MPT_IMPL_UNIFORM_SAMPLER_SO2_HPP
#define MPT_IMPL_UNIFORM_SAMPLER_SO2_HPP

#include <random>
#include <cmath>
#include "constants.hpp"
#include "../so2_space.hpp"
#include "../box_bounds.hpp"

namespace unc::robotics::mpt::impl {
    template <typename T, int p, int dim = nigh::metric::Space<T, SO2<p>>::kDimensions>
    struct SO2UniformSampler {
    private:
        using Space = nigh::metric::Space<T, SO2<p>>;
        using Scalar = typename Space::Distance;

    public:
        SO2UniformSampler(const Space&) {
        }

        template <typename RNG>
        T operator() (RNG& rng) const {
            std::uniform_real_distribution<Scalar> dist(-impl::PI<Scalar>, impl::PI<Scalar>);
            T q;
            for (int i=0 ; i < dim ; ++i)
                Space::coeff(q, i) = dist(rng);
            return q;
        }
    };
    
    template <typename T, int p>
    struct SO2UniformSampler<T, p, -1> {
    private:
        using Space = nigh::metric::Space<T, SO2<p>>;
        using Scalar = typename Space::Distance;

        int dimensions_;

    public:
        SO2UniformSampler(const Space& space) : dimensions_(space.dimensions()) {
        }

        template <typename RNG>
        T operator() (RNG& rng) const {
            std::uniform_real_distribution<Scalar> dist(-impl::PI<Scalar>, impl::PI<Scalar>);
            T q;
            for (int i=0 ; i < dimensions_ ; ++i)
                Space::coeff(q, i) = dist(rng);
            return q;
        }
    };
}

namespace unc::robotics::mpt {
    template <class T, int p, int dim>
    auto measure(
        const impl::SO2UniformSampler<T, p, dim>&,
        const nigh::metric::Space<T, SO2<p>>& space)
    {
        using Scalar = typename nigh::metric::Space<T, SO2<p>>::Distance;
        return std::pow(2 * impl::PI<Scalar>, space.dimensions());
    }
    
    template <typename T, int p>
    struct UniformSampler<Space<T, SO2<p>>, Unbounded> 
        : impl::SO2UniformSampler<T, p>
    {
        using Base = impl::SO2UniformSampler<T, p>;

        UniformSampler(const Space<T, SO2<p>>& space, Unbounded = Unbounded{}) 
            : Base(space)
        {
        }
    };

    // template <typename T, int p, typename S, int dim>
    // struct UniformSampler<Space<T, SO2<p>>, BoxBounds<S, dim>>
    //     : impl::SO2UniformSampler<T, p, Space<T, SO2<p>>::kDimensions>
    // {
    //     using SO2UniformSampler<T, p, Space<T, SO2<p>>::kDimensions>::SO2UniformSampler;
    // };
}

#endif
