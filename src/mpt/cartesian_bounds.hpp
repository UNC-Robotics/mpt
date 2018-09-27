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
#ifndef MPT_CARTESIAN_BOUNDS_HPP
#define MPT_CARTESIAN_BOUNDS_HPP

#include "unbounded.hpp"
#include <tuple>

namespace unc::robotics::mpt {
    template <typename ... Bounds>
    class CartesianBounds : public std::tuple<Bounds...> {
        using Base = std::tuple<Bounds...>;

        template <std::size_t I, std::size_t J, typename Args, typename ... Pack>
        static decltype(auto) expand1(Args&& args, Pack&& ... pack) {
            using IType = std::tuple_element_t<I, Base>;
            static constexpr bool unbounded = std::is_same_v<Unbounded, IType>;

            if constexpr (unbounded && J == std::tuple_size_v<Args>) {
                return expand<I+1, J>(std::forward<Args>(args), std::forward<Pack>(pack)..., Unbounded{});
            } else if constexpr (unbounded && !std::is_same_v<Unbounded, std::decay_t<std::tuple_element_t<J, Args>>>) {
                return expand<I+1, J>(std::forward<Args>(args), std::forward<Pack>(pack)..., Unbounded{});
            } else {
                return expand<I+1,J+1>(std::forward<Args>(args), std::forward<Pack>(pack)..., std::get<J>(args));
            }
        }

        template <std::size_t I, std::size_t J, typename Args, typename ... Pack>
        static decltype(auto) expand(Args&& args, Pack&& ... pack) {
            if constexpr (I == sizeof...(Bounds)) {
                return Base(std::forward<Pack>(pack)...);
            } else {
                return expand1<I, J>(std::forward<Args>(args), std::forward<Pack>(pack)...);
            }
        }

    public:
        // This constructor expects constructor arguments for all
        // contained bounds, except 'Unbounded'.
        template <typename ... Args>
        CartesianBounds(Args&& ... args)
            : Base(expand<0, 0>(std::forward_as_tuple(std::forward<Args>(args)...)))
        {
        }
    };

    // The following are specializations that allow cartesian bounds
    // with a single unbounded sub-bound to be constructed with the
    // arguments to the single bound.  The two following are temporary
    // solutions, since we can support such construction on a more
    // genera case without using specializations.
    template <typename Bound>
    struct CartesianBounds<Unbounded, Bound> : std::tuple<Unbounded, Bound> {
        using Base = std::tuple<Unbounded, Bound>;
    public:
        template <typename ... Args>
        CartesianBounds(Args&& ... args)
            : Base(Unbounded{}, Bound(std::forward<Args>(args)...))
        {
        }

        template <typename ... Args>
        CartesianBounds(Unbounded, Args&& ... args)
            : Base(Unbounded{}, Bound(std::forward<Args>(args)...))
        {
        }
    };

    namespace impl {
        template <typename ... Args>
        struct last_argument { using type = void; };
        template <typename Arg>
        struct last_argument<Arg> { using type = Arg; };
        template <typename First, typename ... Rest>
        struct last_argument<First, Rest...> : last_argument<Rest...> {};
        template <typename ... Args>
        using last_argument_t = typename last_argument<Args...>::type;
    }
}

namespace std {
    template <std::size_t I, typename ... Bounds>
    struct tuple_element<I, unc::robotics::mpt::CartesianBounds<Bounds...>> : tuple_element<I, std::tuple<Bounds...>> {};
}

#endif
