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
#ifndef MPT_IMPL_SCENARIO_BOUNDS_HPP
#define MPT_IMPL_SCENARIO_BOUNDS_HPP

#include "../unbounded.hpp"
#include <type_traits>

namespace unc::robotics::mpt::impl {

    // Extracts the bounds type of the scenario.  The bounds type is
    // determined by the return type of the bounds() method on the
    // scenario.  If no such method exists, then the default is Unbounded.
    template <typename Scenario, class = void>
    struct scenario_bounds {
        using type = Unbounded;
    };

    template <typename Scenario>
    struct scenario_bounds<Scenario, std::void_t<decltype( std::declval<const Scenario&>().bounds() )>> {
        using type = std::decay_t<decltype( std::declval<const Scenario&>().bounds() )>;
    };

    template <typename Scenario>
    using scenario_bounds_t = typename scenario_bounds<Scenario>::type;

    template <typename Scenario>
    constexpr bool scenario_has_bounds_v = !std::is_same_v<Unbounded, scenario_bounds_t<Scenario>>;
}

#endif
