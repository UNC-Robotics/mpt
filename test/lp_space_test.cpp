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

#include <mpt/lp_space.hpp>
#include "test.hpp"

TEST(distance) {
    using Space = unc::robotics::mpt::LPSpace<double, 3, 2>;

    static_assert(std::is_same_v<Space::Type, Eigen::Vector3d>);

    typename Space::Type a(1,2,3);
    typename Space::Type b(1,0,-1);

    Space space;

    EXPECT(space.distance(a, b)) == std::sqrt(0.0 + 2.0*2.0 + 4.0*4.0);
}


TEST(interpolate) {
    using namespace unc::robotics::mpt;
    using Space = LPSpace<double, 3, 2>;

    Space space;

    typename Space::Type a(1,2,3);
    typename Space::Type b(1,0,-1);

    typename Space::Type c = interpolate(space, a, b, 0.1);

    EXPECT(c[0]) == 1.0;
    EXPECT(c[1]) == 1.8;
    EXPECT(c[2]) == 2.6;
}
