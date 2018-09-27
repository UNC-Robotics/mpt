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

#include <mpt/so3_space.hpp>
#include "test.hpp"

TEST(distance) {
    using Space = unc::robotics::mpt::SO3Space<double>;

    static_assert(std::is_same_v<Space::Type, Eigen::Quaterniond>);

    typename Space::Type a, b;

    Eigen::Vector3d axis{-1,2,3};
    axis.normalize();
    a = Eigen::AngleAxisd(-1.0, axis);
    b = Eigen::AngleAxisd(2.0, axis);

    Space space;

    // currently the distance metric is halved since the distance is
    // computed acos(abs(dot(a,b))) without an additional 2* multiplier.
    EXPECT(std::abs(space.distance(a, b) - 3.0/2)) < 1e-10;
}

TEST(interpolate) {
    using namespace unc::robotics::mpt;

    using Space = SO3Space<double>;
    using State = typename Space::Type;

    Space space;
    State a, b, c;

    Eigen::Vector3d axis{-1,2,3};
    axis.normalize();
    a = Eigen::AngleAxisd(0.5, axis);
    b = Eigen::AngleAxisd(2.5, axis);

    c = interpolate(space, a, b, 0.1);

    Eigen::AngleAxisd aa;
    aa = c;

    EXPECT((aa.axis() - axis).squaredNorm()) < 1e-15;
    EXPECT(std::abs(aa.angle() - (0.5 + 2*0.1))) < 1e-10;
}
