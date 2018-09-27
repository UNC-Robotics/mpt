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
#ifndef MPT_SO3_SPACE_HPP
#define MPT_SO3_SPACE_HPP

#include <nigh/so3_space.hpp>
#include "impl/metrics.hpp"

namespace unc::robotics::mpt {
    template <typename T>
    T interpolate(
        const Space<T, SO3>& space, const T& a, const T& b,
        typename Space<T, SO3>::Distance t)
    {
        using S = Space<T, SO3>;
        using Scalar = typename S::Distance;
        // TODO: for Eigen types, we can do this:
        // return a.slerp(t, b);

        Scalar d = S::coeff(a, 0) * S::coeff(b, 0)
            + S::coeff(a, 1) * S::coeff(b, 1)
            + S::coeff(a, 2) * S::coeff(b, 2)
            + S::coeff(a, 3) * S::coeff(b, 3);
        Scalar ad = std::abs(d);
        Scalar s0, s1;

        if (d >= Scalar(1) - std::numeric_limits<Scalar>::epsilon()) {
            s0 = Scalar(1) - t;
            s1 = t;
        } else {
            Scalar theta = std::acos(ad);
            Scalar sinTheta = std::sin(theta);

            s0 = std::sin( (Scalar(1) - t) * theta) / sinTheta;
            s1 = std::sin( (t * theta) ) / sinTheta;
        }

        if (d < 0)
            s1 = -s1;

        T q;

        S::coeff(q, 0) = s0 * S::coeff(a, 0) + s1 * S::coeff(b, 0);
        S::coeff(q, 1) = s0 * S::coeff(a, 1) + s1 * S::coeff(b, 1);
        S::coeff(q, 2) = s0 * S::coeff(a, 2) + s1 * S::coeff(b, 2);
        S::coeff(q, 3) = s0 * S::coeff(a, 3) + s1 * S::coeff(b, 3);

        return q;
    }
}

#endif
