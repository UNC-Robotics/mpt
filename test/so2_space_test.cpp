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

#include <mpt/so2_space.hpp>
#include "test.hpp"

TEST(distance_scalar) {
    using Space = unc::robotics::mpt::SO2Space<double>;

    static_assert(std::is_same_v<Space::Type, double>);

    Space space;

    EXPECT(space.distance(1.0, 1.0)) == 0;
    EXPECT(space.distance(0.0, 2.0)) == 2.0;
    EXPECT(space.distance(2.0, 0.0)) == 2.0;
    EXPECT(space.distance(-1.0, 3.0)) == 2*M_PI - 4.0;
    EXPECT(space.distance(3.0, -1.0)) == 2*M_PI - 4.0;
}


TEST(distance_lp1) {
    using Space = unc::robotics::mpt::SO2LPSpace<double, 3, 1>;

    static_assert(std::is_same_v<Space::Type, Eigen::Vector3d>);

    typename Space::Type a(1,2,3);
    typename Space::Type b(1,0,-1);

    Space space;

    EXPECT(space.distance(a, b)) == 0.0 + 2.0 + 2*M_PI - 4;
}

TEST(interpolate_scalar) {
    using namespace unc::robotics::mpt;
    using Space = unc::robotics::mpt::SO2Space<double>;

    Space space;

    EXPECT(interpolate(space, 1.0, 1.0, 0.0)) == 1.0; 
    EXPECT(interpolate(space, 1.0, 1.0, 3.0)) == 1.0; 

    EXPECT(interpolate(space, 1.0, 2.0, 0.5)) == 1.5; 
    EXPECT(interpolate(space, 1.0, 2.0, -3.0)) == -2.0; 
    EXPECT(interpolate(space, -1.0, 2.0, 4.0)) == 11.0 - 4*M_PI; 

    EXPECT(interpolate(space, 5*M_PI/6, -5*M_PI/6, 1.0)) == -5*M_PI/6; 
    EXPECT(interpolate(space, 5*M_PI/6, -5*M_PI/6, 2.0)) == -3*M_PI/6; 
    // EXPECT(interpolate(space, -5*M_PI/6, 5*M_PI/6, -2.0)) == -1*M_PI/6;
    EXPECT(interpolate(space, -5*M_PI/6, 5*M_PI/6, 2.0)) == 3*M_PI/6;
}
