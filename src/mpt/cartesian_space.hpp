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
#ifndef MPT_CARTESIAN_SPACE_HPP
#define MPT_CARTESIAN_SPACE_HPP

#include <nigh/cartesian_space.hpp>
#include "impl/metrics.hpp"

namespace unc::robotics::mpt::impl {
    template <typename T, typename M, std::size_t ... I>
    T cartesian_interpolate(
        const Space<T, M>& space, const T& a, const T& b,
        typename Space<T, M>::Distance t,
        std::index_sequence<I...>)
    {
        T q;
        ((cartesian_state_element<I, T>::get(q) = interpolate(
            std::get<I>(space),
            cartesian_state_element<I, T>::get(a),
            cartesian_state_element<I, T>::get(b),
            t)), ...);
        return q;
    }
}

namespace unc::robotics::mpt {
    template <typename T, typename ... M>
    T interpolate(
        const Space<T, Cartesian<M...>>& space, const T& a, const T& b,
        typename Space<T, Cartesian<M...>>::Distance t)
    {
        return impl::cartesian_interpolate(space, a, b, t, std::index_sequence_for<M...>{});
    }
}

#endif
