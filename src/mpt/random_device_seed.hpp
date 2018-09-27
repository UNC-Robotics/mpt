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

#ifndef MPT_RANDOM_DEVICE_SEED_HPP_
#define MPT_RANDOM_DEVICE_SEED_HPP_

#include <array>
#include <random>
#include <algorithm>

namespace unc::robotics::mpt {

    /**
     * An object that nominally conforms to the SeedSequence concept,
     * and can be used to seed pseudo-random number generators from a
     * different of randomness.  It further uses std::seed_seq to help
     * distribute the data in case the underlying source of randomness
     * is poorly distributed.
     *
     * This does not implement the SeedSequence constructors that take
     * sequence arguments, nor does it implement the param() method,
     * as its result would be useless.
     *
     * @param _size is the maximum amount of entropy to use from the
     * random device.  By default the size it reports is 624 which is
     * exactly enough for a MersenneTwister19937.
     *
     * @param _RDev is the underlying source of randomness to use.
     */
    template <std::size_t _size = 624, typename _RDev = std::random_device>
    class RandomDeviceSeed {
        mutable _RDev rdev_;

    public:
        typedef std::uint32_t result_type;

        template <typename ... _Args>
        RandomDeviceSeed(_Args&& ... args)
            : rdev_(std::forward<_Args>(args)...)
        {
        }

        template <class _RandomIt>
        void generate(_RandomIt begin, _RandomIt end) const {
            std::size_t n = std::min(_size, std::size_t(std::distance(begin, end)));
            std::array<result_type, _size> data;
            std::uniform_int_distribution<std::uint32_t> dist32;
            std::generate_n(data.begin(), n, [&] { return dist32(rdev_); });
            std::seed_seq seq(data.begin(), data.begin() + n);
            seq.generate(begin, end);
        }

        std::size_t size() const {
            return _size;
        }
    };

    // template <typename _RNG>
    // void initRNG() {
    //     static constexpr std::uint32_t nStates = _RNG::state_size * _RNG::word_size / 32;
    //     std::random_device rdev;
    //     std::array<std::uint32_t, nStates> seedData;
    //     std::uniform_int_distribution<std::uint32_t> dist32;
    //     std::generate_n(seedData.begin(), nStates, [&] { return dist32(rdev); });
    //     std::seed_seq seedSeq(seedData.begin(), seedData.end());
    //     _RNG rng(seedSeq);
    // }

}

#endif // MPT_RANDOM_DEVICE_SEED_HPP_
