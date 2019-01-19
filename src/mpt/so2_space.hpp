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
#ifndef MPT_SO2_SPACE_HPP
#define MPT_SO2_SPACE_HPP

#include <nigh/so2_space.hpp>
#include "impl/metrics.hpp"
#include <iostream>
namespace unc::robotics::mpt {
    template <typename T, int p>
    T interpolate(
        const Space<T, SO2<p>>& space, const T& a, const T& b,
        typename Space<T, SO2<p>>::Distance d)
    {
        using namespace unc::robotics::nigh::impl;
        using S = Space<T, SO2<p>>;
        using Scalar = typename S::Distance;
        T q;
        for (unsigned i=0; i < space.dimensions(); ++i) {
            Scalar ccwDist = so2::ccwDist(S::coeff(a, i), S::coeff(b, i));
            if (ccwDist < PI<Scalar>) {
                // angular distance is in ccw direction from a to b
                S::coeff(q, i) = so2::bound(S::coeff(a, i) + ccwDist * d); 
            } else {
                // angular distance is in cw direction from a to b
                Scalar cwDist = 2*PI<Scalar> - ccwDist;
                S::coeff(q, i) = so2::bound(S::coeff(a, i) - cwDist * d); 
            }
        }
        return q;
    }
}
#endif
