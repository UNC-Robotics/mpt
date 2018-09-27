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
#ifndef MPT_SE2_SPACE_HPP
#define MPT_SE2_SPACE_HPP

#include "lp_space.hpp"
#include "so2_space.hpp"
#include "scaled_space.hpp"
#include "cartesian_space.hpp"

namespace unc::robotics::mpt {
    template <typename Scalar>
    class SE2State
        : public std::tuple<Eigen::Matrix<Scalar, 2, 1>, Scalar>
    {
        using Base = std::tuple<Eigen::Matrix<Scalar, 2, 1>, Scalar>;

    public:
        using Base::Base;

        Scalar& rotation() { return std::get<1>(*this); }
        const Scalar& rotation() const { return std::get<1>(*this); }

        Eigen::Matrix<Scalar, 2, 1>& translation() { return std::get<0>(*this); }
        const Eigen::Matrix<Scalar, 2, 1>& translation() const { return std::get<0>(*this); }
    };

    namespace impl {
        template <typename S, std::intmax_t so2, std::intmax_t l2>
        struct se2_space_selector {
            using type = Space<SE2State<S>, Cartesian<Scaled<L2, std::ratio<l2>>, Scaled<SO2<>, std::ratio<so2>>>>;
        };

        template <typename S, std::intmax_t so2>
        struct se2_space_selector<S, so2, 1> {
            using type = Space<SE2State<S>, Cartesian<L2, Scaled<SO2<>, std::ratio<so2>>>>;
        };

        template <typename S, std::intmax_t l2>
        struct se2_space_selector<S, 1, l2> {
            using type = Space<SE2State<S>, Cartesian<Scaled<L2, std::ratio<l2>>, SO2<>>>;
        };

        template <typename S>
        struct se2_space_selector<S, 1, 1> {
            using type = Space<SE2State<S>, Cartesian<L2, SO2<>>>;
        };
    }

    template <typename Scalar, std::intmax_t so2wt, std::intmax_t l2wt>
    using SE2Space = typename impl::se2_space_selector<Scalar, so2wt, l2wt>::type;
}

namespace std {
    template <typename Scalar>
    struct tuple_element<0, unc::robotics::mpt::SE2State<Scalar>> {
        using type = Eigen::Matrix<Scalar, 2, 1>;
    };

    template <typename Scalar>
    struct tuple_element<1, unc::robotics::mpt::SE2State<Scalar>> {
        using type = Scalar;
    };
}

#endif
