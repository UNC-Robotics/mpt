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
#ifndef MPT_SE3_SPACE_HPP
#define MPT_SE3_SPACE_HPP

#include "lp_space.hpp"
#include "so3_space.hpp"
#include "scaled_space.hpp"
#include "cartesian_space.hpp"
#include <ostream> // TODO: can we make this optional, along with its usage?
// #include <nigh/se3_space.hpp>

namespace unc::robotics::mpt {
    // MPT provides a simple state wrapper class for SE3State that has
    // a rotation and translation element.  It inherits from
    // std::tuple to require minimal shims to work with nigh's
    // nearest-neighbor searching.  The only requirement is that we
    // provide specializations of std::tuple_element.
    template <typename Scalar>
    class SE3State
        : public std::tuple<Eigen::Quaternion<Scalar>, Eigen::Matrix<Scalar, 3, 1>>
    {
        using Base = std::tuple<Eigen::Quaternion<Scalar>, Eigen::Matrix<Scalar, 3, 1>>;

        Base& tuple() { return *this; }
        const Base& tuple() const { return *this; }

    public:
        using Base::Base;

        Eigen::Quaternion<Scalar>& rotation() {
            return std::get<0>(*this);
        }

        const Eigen::Quaternion<Scalar>& rotation() const {
            return std::get<0>(*this);
        }

        Eigen::Matrix<Scalar, 3, 1>& translation() {
            return std::get<1>(*this);
        }

        const Eigen::Matrix<Scalar, 3, 1>& translation() const {
            return std::get<1>(*this);
        }

        template <typename Char, typename Traits>
        friend decltype(auto)
        operator << (std::basic_ostream<Char, Traits>& out, const SE3State& q) {
            return out << "{t=[" << q.translation().transpose()
                       << "], r=[" << q.rotation().coeffs().transpose()
                       << "]}";
        }
    };

    namespace impl {
        template <typename S, std::intmax_t so3, std::intmax_t l2>
        struct se3_space_selector {
            using type = Space<SE3State<S>, Cartesian<Scaled<SO3, std::ratio<so3>>, Scaled<L2, std::ratio<l2>>>>;
        };

        template <typename S, std::intmax_t so3>
        struct se3_space_selector<S, so3, 1> {
            using type = Space<SE3State<S>, Cartesian<Scaled<SO3, std::ratio<so3>>, L2>>;
        };

        template <typename S, std::intmax_t l2>
        struct se3_space_selector<S, 1, l2> {
            using type = Space<SE3State<S>, Cartesian<SO3, Scaled<L2, std::ratio<l2>>>>;
        };

        template <typename S>
        struct se3_space_selector<S, 1, 1> {
            using type = Space<SE3State<S>, Cartesian<SO3, L2>>;
        };
    }

    template <typename Scalar, std::intmax_t so3wt = 1, std::intmax_t l2wt = 1>
    using SE3Space = typename impl::se3_space_selector<Scalar, so3wt, l2wt>::type;
}

namespace std {
    template <typename Scalar>
    struct tuple_element<0, unc::robotics::mpt::SE3State<Scalar>> {
        using type = Eigen::Quaternion<Scalar>;
    };

    template <typename Scalar>
    struct tuple_element<1, unc::robotics::mpt::SE3State<Scalar>> {
        using type = Eigen::Matrix<Scalar, 3, 1>;
    };
}

#endif
