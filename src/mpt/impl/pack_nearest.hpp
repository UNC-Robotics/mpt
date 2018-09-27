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
#ifndef MPT_IMPL_PACK_NEAREST_HPP
#define MPT_IMPL_PACK_NEAREST_HPP

#include <nigh/auto_strategy.hpp>

namespace unc::robotics::mpt::impl {
    template <typename ... Pack>
    struct pack_nearest {
        using type = void;
    };

    template <typename First, typename ... Rest>
    struct pack_nearest<First, Rest...> : pack_nearest<Rest...> {};

    template <typename ... Pack>
    using pack_nearest_t = typename pack_nearest<Pack...>::type;

    template <typename ... Rest>
    struct pack_nearest<nigh::Linear, Rest...> {
        using type = nigh::Linear;
        static_assert(
            std::is_void_v<pack_nearest_t<Rest...>>,
            "multiple nearest neighbor strategies");
    };

    template <std::size_t batchSize, typename ... Rest>
    struct pack_nearest<nigh::KDTreeBatch<batchSize>, Rest...> {
        using type = nigh::KDTreeBatch<batchSize>;
        static_assert(
            std::is_void_v<pack_nearest_t<Rest...>>,
            "multiple nearest neighbor strategies");
    };

    template <
        unsigned degree, unsigned minDegree, unsigned maxDegree,
        unsigned maxNumPtsPerLeaf, unsigned removedCacheSize,
        bool rebalancing,
        typename ... Rest>
    struct pack_nearest<
        nigh::GNAT<degree, minDegree, maxDegree, maxNumPtsPerLeaf, removedCacheSize, rebalancing>,
        Rest...>
    {
        using type = nigh::GNAT<degree, minDegree, maxDegree, maxNumPtsPerLeaf, removedCacheSize, rebalancing>;
        static_assert(
            std::is_void_v<pack_nearest_t<Rest...>>,
            "multiple nearest neighbor strategies");
    };
}

#endif
