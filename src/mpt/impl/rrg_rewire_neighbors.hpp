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
#ifndef MPT_IMPL_RRG_REWIRE_HPP
#define MPT_IMPL_RRG_REWIRE_HPP

#include "../planner_tags.hpp"

namespace unc::robotics::mpt::impl {
    template <class Strategy, class Space>
    struct RRGRewireNeighbors;

    template <class Space>
    struct RRGRewireNeighbors<rewire_k_nearest, Space> {
        using State = typename Space::Type;
        using Scalar = typename Space::Distance;
        
        Scalar kRRG_;

        unsigned rewireCount(std::size_t n) const {
            return std::ceil(kRRG_ * std::log(Scalar(n + 1)));
        }
        
    public:
        template <class Sampler>
        RRGRewireNeighbors(const Sampler&, const Space& space, Scalar factor = 1.0)
            : kRRG_(factor * E<Scalar> * (1 + 1/static_cast<Scalar>(space.dimensions())))
        {
        }

        template <class Neighbors, class NN>
        void operator() (Neighbors& nbh, const NN& nn, const State& q) const {
            nn.nearest(nbh, q, rewireCount(nn.size()));
        }

        class Stats {
            friend class RRGRewireNeighbors;
            
            unsigned rewireCount_;

            Stats(unsigned c)
                : rewireCount_{c}
            {
            }
            
        public:
            template <class Char, class Traits>
            friend auto& operator << (
                std::basic_ostream<Char, Traits>& out,
                const Stats& stats)
            {
                return out << stats.rewireCount_ << " nearest neighbors";
            }
        };

        Stats stats(std::size_t n) const {
            return Stats{rewireCount(n)};
        }
    };

    template <class Space>
    struct RRGRewireNeighbors<rewire_r_nearest, Space> {
        using State = typename Space::Type;
        using Scalar = typename Space::Distance;
        
        Scalar invDim_;
        Scalar rRRG_;

        static Scalar unitNBallMeasure(unsigned dim) {
            return std::pow(std::sqrt(PI<Scalar>), static_cast<Scalar>(dim))
                / std::tgamma(static_cast<Scalar>(dim) / 2 + 1);
        }

        Scalar rewireRadius(std::size_t n) const {
            ++n;
            return rRRG_ * std::pow(std::log(n) / n, invDim_);
        }
        
    public:
        template <class Sampler>
        RRGRewireNeighbors(const Sampler& sampler, const Space& space, Scalar factor = 1.0)
            : invDim_(1 / Scalar(space.dimensions()))
            , rRRG_(
                factor *
                std::pow(
                    2 * (1 + invDim_) * measure(sampler, space) / unitNBallMeasure(space.dimensions()),
                    invDim_))
        {
        }

        template <class Neighbors, class NN>
        void operator() (Neighbors& nbh, const NN& nn, const State& q) const {
            Scalar r = rewireRadius(nn.size());
            nn.nearest(nbh, q, std::numeric_limits<std::size_t>::max(), r);
        }

        class Stats {
            friend class RRGRewireNeighbors;
            
            Scalar radius_;
            Stats(Scalar r)
                : radius_{r}
            {
            }
            
        public:
            template <class Char, class Traits>
            friend auto& operator << (
                std::basic_ostream<Char, Traits>& out,
                const Stats& stats)
            {
                return out << "nearest neighbors in radius " << stats.radius_;
            }
        };

        Stats stats(std::size_t n) const {
            return Stats{rewireRadius(n)};
        }
    };
}

#endif
