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

#include <mpt/impl/packs.hpp>
#include "test.hpp"

TEST(pack_find_v) {
    using namespace unc::robotics::mpt::impl;

    EXPECT((pack_find_v<std::is_integral, void, std::string, int, int*>)) == true;
}

TEST(pack_find_t) {
    using namespace unc::robotics::mpt::impl;

    EXPECT((std::is_same_v<int, pack_find_t<std::is_integral, void, std::string, int, int*>>)) == true;
}

TEST(pack_default) {
    using namespace unc::robotics::mpt::impl;

    EXPECT((std::is_same_v<long, pack_find_t<std::is_integral, long, std::string, void>>)) == true;
}

TEST(pack_contains_v) {
    using namespace unc::robotics::mpt::impl;
    EXPECT((pack_contains_v<int, std::string, float>)) == false;
    EXPECT((pack_contains_v<int, int, std::string>)) == true;
    EXPECT((pack_contains_v<int, std::string, int>)) == true;
}

template <bool v>
struct test_bool {};

TEST(pack_bool_tag_v) {
    using namespace unc::robotics::mpt::impl;
    EXPECT((pack_bool_tag_v<test_bool, false, int, bool>)) == false;
    EXPECT((pack_bool_tag_v<test_bool, false, test_bool<true>, bool>)) == true;
    EXPECT((pack_bool_tag_v<test_bool, false, test_bool<false>, bool>)) == false;
}
