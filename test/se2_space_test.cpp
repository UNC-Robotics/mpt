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

#include <mpt/se2_space.hpp>
#include "test.hpp"

// template <typename T>
// constexpr std::string_view type_name() {
//     std::string_view p = __PRETTY_FUNCTION__;
//     return std::string_view(p.data() + 34, p.size() - 34 - 1);
// }

template <typename Scalar, std::intmax_t so2w, std::intmax_t l2w>
void testSE2dist() {
    using namespace unc::robotics::mpt;
    using Space = SE2Space<Scalar, so2w, l2w>;
    using Rot2 = Scalar;
    using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

    // std::cout << type_name<Space>() << std::endl;

    static_assert(std::is_same_v<SE2State<Scalar>, typename Space::Type>);

    Space space;

    typename Space::Type a, b;

    std::get<Rot2>(a) = -1.0;
    std::get<1>(b) = 2.0;

    std::get<0>(a) << 2, 3;
    std::get<Vec2>(b) << 0, -1;

    EXPECT(space.distance(a, b)) == 3.0 * so2w + std::sqrt(0.0 + 2.0*2.0 + 4.0*4.0) * l2w;
}


TEST(distance_1_1) {
    testSE2dist<double,1,1>();
}

TEST(distance_5_2) {
    testSE2dist<double,5,2>();
}

TEST(distance_11_1) {
    testSE2dist<double,11,1>();
}

TEST(distance_1_13) {
    testSE2dist<double,1,13>();
}
