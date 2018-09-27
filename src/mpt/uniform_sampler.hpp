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
#ifndef MPT_UNIFORM_SAMPLER_HPP
#define MPT_UNIFORM_SAMPLER_HPP

namespace unc::robotics::mpt {
    template <typename Space, typename Bounds>
    struct UniformSampler;
}

#include "impl/uniform_sampler_so3.hpp"
#include "impl/uniform_sampler_so2.hpp"
#include "impl/uniform_sampler_scaled.hpp"
#include "impl/uniform_sampler_cartesian.hpp"
#include "impl/uniform_sampler_lp.hpp"

//     template <typename T>
//     struct UniformSampler<MetricSpace<T, SO3Metric>>
//         : impl::SO3UniformSampler<T>
//     {
//         using impl::SO3UniformSampler<T>::SO3UniformSampler;
//     };

//     template <typename T, int p>
//     struct UniformSampler<MetricSpace<T, SO2Metric<p>>>
//         : impl::SO2UniformSampler<T, p, typename MetricSpace<T, SO2Metric<p>>::kDimensions>
//     {
//         using SO2UniformSampler<T, p, typename MetricSpace<T, SO2Metric<p>>::kDimensions>::SO2UniformSampler;
//     };

//     template <typename T, typename M, typename W>
//     struct UniformSampler<MetricSpace<T, ScaledMetric<M, W>>>
//         : UniformSampler<T, M>
//     {
//         using UniformSampler<T, M>::UniformSampler;
//     };

//     template <typename T, int p, typename S, int dim>
//     struct UniformSampler<BoundedSpace<T, LPMetric<p>, BoxBounds<S, dim>>>
//         : BoxUniformSampler<T, S, dim>
//     {
//         using BoxUniformSampler<T, S, dim>::BoxUniformSampler;
//     };

//     template <typename T, typename ... M>
//     struct UniformSampler<MetricSpace<T, CartesianMetric<M...>>>
//         : impl::CartesianUniformSampler<T, CartesianMetric<M...>, std::index_sequence_for<M...>>
//     {
//         using impl::CartesianUniformSampler<T, CartesianMetric<M...>, std::index_sequence_for<M...>>::CartesianUniformSampler;
//     };
// }

#endif
