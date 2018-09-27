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

#include <mpt/box_bounds.hpp>
#include <mpt/unbounded.hpp>
#include <mpt/cartesian_bounds.hpp>
#include "test.hpp"

TEST(box_bounds) {
    using namespace unc::robotics::mpt;
    BoxBounds<double, 3> bounds(Eigen::Vector3d(0,1,2), Eigen::Vector3d(10, 12, 14));

    EXPECT(bounds.min()[1]) == 1.0;
    EXPECT(bounds.max()[2]) == 14.0;
}

TEST(unbounded) {
    using namespace unc::robotics::mpt;
    Unbounded unbounded;

    EXPECT(sizeof(unbounded)) <= 1;
}

TEST(cartesian_bounds_2_boxes) {
    using namespace unc::robotics::mpt;
    using Vec2 = Eigen::Vector2d;
    using Vec3 = Eigen::Vector3d;
    CartesianBounds<BoxBounds<double, 3>, BoxBounds<double, 2>> bounds(
        BoxBounds<double, 3>(Vec3(1,2,3), Vec3(2,4,6)),
        BoxBounds<double, 2>(Vec2(4,5), Vec2(8,7)));

    EXPECT(std::get<0>(bounds).min()[2]) == 3.0;
    EXPECT(std::get<1>(bounds).max()[0]) == 8.0;
}

TEST(cartesian_bounds_se3) {
    using namespace unc::robotics::mpt;
    using Vec3 = Eigen::Vector3d;

    // CartesianBound's constructor makes Unbounded arguments
    // optional.
    CartesianBounds<Unbounded, BoxBounds<double, 3>> bounds(
        BoxBounds<double, 3>(Vec3(1,2,3), Vec3(2,4,6)));

    CartesianBounds<Unbounded, BoxBounds<double, 3>> bounds2(
        Unbounded{},
        BoxBounds<double, 3>(Vec3(1,2,3), Vec3(2,4,6)));

    CartesianBounds<Unbounded, BoxBounds<double, 3>> bounds3(
        Vec3(1,2,3), Vec3(2,4,6));

    EXPECT(sizeof(bounds)) == sizeof(BoxBounds<double, 3>);
    EXPECT(std::get<1>(bounds).max()[2]) == 6.0;
    EXPECT(std::get<1>(bounds2).max()[2]) == 6.0;
    EXPECT(std::get<1>(bounds3).max()[2]) == 6.0;
}

TEST(cartesian_bounds_se3r) {
    using namespace unc::robotics::mpt;
    using Vec3 = Eigen::Vector3d;

    // CartesianBound's constructor makes Unbounded arguments
    // optional.
    CartesianBounds<BoxBounds<double, 3>, Unbounded> bounds(
        BoxBounds<double, 3>(Vec3(1,2,3), Vec3(2,4,6)));

    CartesianBounds<BoxBounds<double, 3>, Unbounded> bounds2(
        BoxBounds<double, 3>(Vec3(1,2,3), Vec3(2,4,6)),
        Unbounded{});

    // CartesianBounds<BoxBounds<double, 3>, Unbounded> bounds3(
    //     Vec3(1,2,3), Vec3(2,4,6));

    EXPECT(sizeof(bounds)) == sizeof(BoxBounds<double, 3>);
    EXPECT(std::get<0>(bounds).max()[2]) == 6.0;
    EXPECT(std::get<0>(bounds2).max()[2]) == 6.0;
}

