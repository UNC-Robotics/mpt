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
#ifndef MPT_BOX_BOUNDS_HPP
#define MPT_BOX_BOUNDS_HPP

#include <Eigen/Dense>
#include <stdexcept>

namespace unc::robotics::mpt {
    template <typename S, int dim>
    class BoxBounds {
        Eigen::Matrix<S, dim, 2> bounds_;

        void checkBounds() {
            if (!bounds_.allFinite())
                throw std::domain_error("non-finite bounds");

            if ((bounds_.col(0).array() > bounds_.col(1).array()).any())
                throw std::invalid_argument("bounds have min > max");
        }

    public:
        BoxBounds(const BoxBounds& bounds) noexcept
            : bounds_(bounds.bounds_)
        {
        }

        template <typename Bounds>
        BoxBounds(const Eigen::DenseBase<Bounds>& bounds)
            : bounds_(bounds)
        {
            checkBounds();
        }

        template <typename Min, typename Max>
        BoxBounds(
            const Eigen::DenseBase<Min>& min,
            const Eigen::DenseBase<Max>& max)
        {
            bounds_.col(0) = min;
            bounds_.col(1) = max;

            checkBounds();
        }

        decltype(auto) min() const {
            return bounds_.col(0);
        }

        decltype(auto) max() const {
            return bounds_.col(1);
        }

        S measure() const {
            return (max() - min()).prod();
        }

        unsigned size() const {
            return bounds_.rows();
        }
    };
}

#endif
