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
#ifndef MPT_IMPL_PACKS_HPP
#define MPT_IMPL_PACKS_HPP

#include <type_traits>

namespace unc::robotics::mpt::impl {

    // pack_find<Matcher<T>, Default, Pack...>
    //
    // test each type in the pack as a template argument to Matcher.
    // If Matcher has a bool value that is true, pack_find will have a
    // type member of the matching type, and constexpr bool value =
    // true.  Conversely, if there is no match, there will be no type
    // member, and there will be a constexpr bool value = false.
    //
    // pack_find_t is a type alias for the type member
    // pack_find_v is a constexpr bool alias for the value member
    //
    // Examples:
    //
    // pack_find_t<std::is_scalar, float, std::string, int, int*> is int
    // pack_find_v<std::is_scalar, float, std::string, int, int*> is true
    // pack_find_t<std::is_void, float, std::string, int, int*> is float (the default)
    // pack_find_v<std::is_void, float, std::string, int, int*> is false
    //
    // pack_find<std::is_scalar, int, float> generates a static_assert
    // error at compile time

    template <template <class> class Matcher, typename Default, typename ... Pack>
    struct pack_find : std::false_type {
        using type = Default;
    };

    template <template <class> class Matcher, typename Default, typename ... Pack>
    using pack_find_t = typename pack_find<Matcher, Default, Pack...>::type;

    template <template <class> class Matcher, typename Default, typename ... Pack>
    constexpr bool pack_find_v = pack_find<Matcher, Default, Pack...>::value;

    template <bool found, template <class> class Matcher, typename Default, typename T, typename ... Pack>
    struct pack_find_helper : std::true_type {
        static_assert(
            !pack_find_v<Matcher, void, Pack...>,
            "multiple matches in parameter pack");
        using type = T;
    };

    template <template <class> class Matcher, typename Default, typename T, typename ... Pack>
    struct pack_find_helper<false, Matcher, Default, T, Pack...> : pack_find<Matcher, Default, Pack...> {};

    template <template <class> class Matcher, typename Default, typename T, typename ... Pack>
    struct pack_find<Matcher, Default, T, Pack...> : pack_find_helper<Matcher<T>::value, Matcher, Default, T, Pack...> {};

    template <typename T>
    struct exact_match {
        template <typename U> struct matcher : std::is_same<T, U> {};
    };

    template <typename T, typename ... Pack>
    struct pack_contains
        : pack_find<exact_match<T>::template matcher, T, Pack...>
    {
    };

    template <typename T, typename ... Pack>
    constexpr bool pack_contains_v = pack_contains<T, Pack...>::value;


    // pack_value_tag finds a matching template tag within a pack.
    template <typename T, template <T> class Tag, T defaultValue, typename ... Pack>
    struct pack_value_tag {
        using type = T;
        static constexpr T value = defaultValue;
    };

    template <typename T, template <T> class Tag, T defaultValue, T matchedValue, typename ... Rest>
    struct pack_value_tag<T, Tag, defaultValue, Tag<matchedValue>, Rest...> {
        using type = T;
        static constexpr T value = matchedValue;
    };

    template <typename T, template <T> class Tag, T defaultValue, typename U, typename ... Rest>
    struct pack_value_tag<T, Tag, defaultValue, U, Rest...> : pack_value_tag<T, Tag, defaultValue, Rest...> {};

    template <template <bool> class Tag, bool defaultValue, typename ... Pack>
    constexpr bool pack_bool_tag_v = pack_value_tag<bool, Tag, defaultValue, Pack...>::value;

    template <template <int> class Tag, int defaultValue, typename ... Pack>
    constexpr int pack_int_tag_v = pack_value_tag<int, Tag, defaultValue, Pack...>::value;

    // template <typename NN, class = void>
    // struct is_nearest_neighbor : std::false_type {};

    // template <typename NN>
    // struct is_nearest_neighbor<NN, std::void_t<nigh::Nigh<T, Space, KeyFn, nigh::Concurrent, NN>>>
    //     : std::true_type {};

    // template <typename Default, typename ... Pack>
    // struct pack_nearest_neighbor { using type = Default; };

    // template <typename Default, typename F, typename ... Rest>
    // struct pack_nearest_neighbor<Default, F, Rest...>
    //     : std::conditional_t<is_nearest_neighbor_v<F>, pack_nearest_neighbor<F>, pack_nearest_neighbor<Default, Rest...>> {};
}

#endif
