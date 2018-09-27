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
#ifndef MPT_PRRT_HPP
#define MPT_PRRT_HPP

#include "planner.hpp"
#include "planner_tags.hpp"
#include "impl/packs.hpp"
#include "impl/pack_nearest.hpp"
#include "impl/nearest_strategy.hpp"
#include "impl/prrt/prrt.hpp"

namespace unc::robotics::mpt {

    namespace impl {
        // this is the actual strategy type for a PRRT planner
        template <int maxThreads, bool reportStats, typename NNStrategy>
        struct PRRTStrategy {};

        // Option parser to generate a PRRTStrategy from a
        // collection of unordered options.
        template <typename ... Options>
        struct PRRTOptions {
            static constexpr int maxThreads = pack_int_tag_v<max_threads, 0, Options...>;
            static constexpr bool reportStats = pack_bool_tag_v<report_stats, false, Options...>;

            using NNStrategy = pack_nearest_t<Options...>;

            using type = PRRTStrategy<maxThreads, reportStats, NNStrategy>;
        };

        template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
        struct PlannerResolver<Scenario, impl::PRRTStrategy<maxThreads, reportStats, NNStrategy>> {
            using type = impl::prrt::PRRT<
                Scenario, maxThreads, reportStats,
                nearest_strategy_t<Scenario, maxThreads, NNStrategy>>;
        };
    }

    // Type alias for a PRRT*-based planner.  The options supported are:
    // - stats reporting
    //    - tag::report_stats<R>  - Reports stats as it plans, where R is false (default) or true.
    // - a nearest neighbor strategy
    //    - nigh::KDTreeBatch<...> - fastest, supports concurrent operation, but does not support arbitrary metrics
    //    - nigh::Linear - slowest, supports concurrent operations, supports arbitrary metrics
    //    - nigh::GNAT<...> - fast, does NOT support concurrent operations, supports metrics for which triangle property holds
    template <typename ... Options>
    using PRRT = typename impl::PRRTOptions<Options...>::type;
}

#endif
