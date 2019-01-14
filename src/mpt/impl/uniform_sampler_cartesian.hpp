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
#ifndef MPT_IMPL_UNIFORM_SAMPLER_CARTESIAN_HPP
#define MPT_IMPL_UNIFORM_SAMPLER_CARTESIAN_HPP

#include <random>
#include <cmath>
#include "constants.hpp"
#include "../cartesian_space.hpp"

namespace unc::robotics::mpt::impl {
    template <std::size_t I, typename T, typename M, typename Bounds>
    using cartesian_uniform_sampler_t = UniformSampler<
        Space<nigh::cartesian_state_element_t<I, T>,
              std::tuple_element_t<I, M>>,
        std::tuple_element_t<I, Bounds>>;

    template <typename T, typename M, typename Bounds, typename Indices>
    struct CartesianUniformSampler;

    template <typename T, typename M, typename Bounds, std::size_t ... I>
    struct CartesianUniformSampler<T, M, Bounds, std::index_sequence<I...>>
        : std::tuple<cartesian_uniform_sampler_t<I, T, M, Bounds>...>
    {
        using Space = nigh::metric::Space<T, M>;

    private:
        using Base = std::tuple<cartesian_uniform_sampler_t<I, T, M, Bounds>...>;

        const Base& tuple() const { return *this; }

    public:
        CartesianUniformSampler(const Space& space, const Bounds& bounds)
            : Base(cartesian_uniform_sampler_t<I, T, M, Bounds>(
                       std::get<I>(space),
                       std::get<I>(bounds))...)
        {
        }

        template <typename RNG>
        T operator() (RNG& rng) const {
            T q;
            ((std::get<I>(q) = std::get<I>(tuple())(rng)), ...);
            return q;
        }
    };

}

namespace unc::robotics::mpt {
    template <typename T, typename ... M, typename Bounds>
    struct UniformSampler<Space<T, Cartesian<M...>>, Bounds>
        : impl::CartesianUniformSampler<T, Cartesian<M...>, Bounds, std::index_sequence_for<M...>>
    {
        using impl::CartesianUniformSampler<T, Cartesian<M...>, Bounds, std::index_sequence_for<M...>>::CartesianUniformSampler;
    };

    template <class T, class M, class Bounds, std::size_t ... I>
    auto measure(
        const impl::CartesianUniformSampler<T, M, Bounds, std::index_sequence<I...>>& sampler,
        const Space<T, M>& space)
    {
        return (measure(std::get<I>(sampler), std::get<I>(space)) * ...);
    }
}

#endif
