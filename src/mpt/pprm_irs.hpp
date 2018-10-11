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
#ifndef MPT_PPRM_IRS_HPP
#define MPT_PPRM_IRS_HPP

#include "planner.hpp"
#include "planner_tags.hpp"
#include "impl/packs.hpp"
#include "impl/pack_nearest.hpp"
#include "impl/nearest_strategy.hpp"
#include "impl/pprm_irs/pprm_irs.hpp"

namespace unc::robotics::mpt {

    namespace impl {
        // this is the actual strategy type for a PPRM planner
        template <int maxThreads, bool keepDense, bool reportStats, typename NNStrategy>
        struct PPRMIRSStrategy {};

        // Option parser to generate a PPRMStrategy from a
        // collection of unordered options.
        template <typename ... Options>
        struct PPRMIRSOptions {
            static constexpr bool reportStats = pack_bool_tag_v<report_stats, false, Options...>;
            static constexpr bool keepDense = pack_bool_tag_v<keep_dense_edges, false, Options...>;
            static constexpr int maxThreads = pack_int_tag_v<max_threads, 0, Options...>;

            using NNStrategy = pack_nearest_t<Options...>;
            using type = PPRMIRSStrategy<maxThreads, keepDense, reportStats, NNStrategy>;
        };

        template <typename Scenario, int maxThreads, bool keepDense, bool reportStats, typename NNStrategy>
        struct PlannerResolver<Scenario, impl::PPRMIRSStrategy<maxThreads, keepDense, reportStats, NNStrategy>> {
            using type = impl::pprm_irs::PPRMIRS<
                Scenario, maxThreads, keepDense, reportStats,
                nearest_strategy_t<Scenario, maxThreads, NNStrategy>>;
        };
    }

    // Type alias for a PPRM+IRS-based planner.
    //
    // PPRM is a parallelized multicore prababilistic roadmap planner.
    // This implements the PRM* (star) variant that is asymptotically
    // optimal.
    //
    // IRS is the incremental roadmap spanner.  The addition of IRS
    // computes a sparse subset of edges in the roadmap creating for
    // an asymptotically near-optimal motion planner.  The dense edges
    // in the roadmap may optionally be kept in the roadmap.  (Note:
    // there is no point to leaving out the sparse edges as that would
    // just be the PRM planner.)
    //
    // The options supported are:
    // - stats reporting
    //    - tag::report_stats<R>  - Reports stats as it plans, where R is false (default) or true.
    // - keep dense edges in the resulting roadmap
    //    - tag::keep_dense_edges<true> - defaults to false
    // - configurable concurrency level
    //    - max_threads<1> to get a non-concurrent planner
    // - a nearest neighbor strategy
    //    - nigh::KDTreeBatch<...> - fastest, supports concurrent operation, but does not support arbitrary metrics
    //    - nigh::Linear - slowest, supports concurrent operations, supports arbitrary metrics
    //    - nigh::GNAT<...> - fast, does NOT support concurrent operations, supports metrics for which triangle property holds
    template <typename ... Options>
    using PPRMIRS = typename impl::PPRMIRSOptions<Options...>::type;
}

#endif
