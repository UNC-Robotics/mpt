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
#ifndef MPT_IMPL_NEAREST_STRATEGY_HPP
#define MPT_IMPL_NEAREST_STRATEGY_HPP

#include "scenario_space.hpp"
#include <type_traits>
#include <nigh/auto_strategy.hpp>

namespace unc::robotics::mpt::impl {
    template <typename Space, typename Concurrency, typename NN>
    struct nearest_strategy_impl {
        // TODO: check that space and concurrency is supported by the
        // specified NN.
        using type = NN;
    };

    template <typename Space, typename Concurrency>
    struct nearest_strategy_impl<Space, Concurrency, void> {
        using type = nigh::auto_strategy_t<Space, Concurrency>;
    };

    // nearest_strategy<...> selects the nearest neighbor strategy
    // based upon the arguments.  The Scenario defines the space,
    // maxThreads is the maximum number of threads, NN is the
    // configured strategy (void = auto detect), and ST is the
    // concurrency model when running single threaded.

    template <typename Scenario, int maxThreads, typename NN, typename ST = nigh::NoThreadSafety>
    struct nearest_strategy : nearest_strategy_impl<
        scenario_space_t<Scenario>,
        std::conditional_t<maxThreads == 1, ST, nigh::Concurrent>,
        NN>
    {
    };

    template <typename Scenario, int maxThreads, typename NN, typename ST = nigh::NoThreadSafety>
    using nearest_strategy_t = typename nearest_strategy<Scenario, maxThreads, NN, ST>::type;

}

#endif
