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

#include <mpt/se3_space.hpp>
#include "test.hpp"

// template <typename T>
// constexpr std::string_view type_name() {
//     std::string_view p = __PRETTY_FUNCTION__;
//     return std::string_view(p.data() + 34, p.size() - 34 - 1);
// }

template <typename Scalar, std::intmax_t so3w, std::intmax_t l2w>
void testSE3dist() {
    using namespace unc::robotics::mpt;
    using Space = SE3Space<Scalar, so3w, l2w>;
    using Quat = Eigen::Quaternion<Scalar>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using AAxs = Eigen::AngleAxis<Scalar>;

    // std::cout << type_name<Space>() << std::endl;

    static_assert(std::is_same_v<SE3State<Scalar>, typename Space::Type>);

    Space space;

    typename Space::Type a, b;

    Vec3 axis{-1,2,3};
    axis.normalize();

    std::get<Quat>(a) = AAxs(-1.0, axis);
    std::get<0>(b) = AAxs(2.0, axis);

    std::get<1>(a) << 1, 2, 3;
    std::get<Vec3>(b) << 1, 0, -1;

    EXPECT(space.distance(a, b)) == 3.0/2 * so3w + std::sqrt(0.0 + 2.0*2.0 + 4.0*4.0) * l2w;
}


TEST(distance_1_1) {
    testSE3dist<double,1,1>();
}

TEST(distance_5_2) {
    testSE3dist<double,5,2>();
}

TEST(distance_11_1) {
    testSE3dist<double,11,1>();
}

TEST(distance_1_13) {
    testSE3dist<double,1,13>();
}

TEST(interpolate) {
    using namespace unc::robotics::mpt;
    using Space = SE3Space<double>;

    Space space;
    typename Space::Type a, b, c;

    Eigen::Vector3d axis{-1, 2, 3};
    axis.normalize();

    std::get<Eigen::Quaterniond>(a) = Eigen::AngleAxis(0.5, axis);
    std::get<Eigen::Quaterniond>(b) = Eigen::AngleAxis(2.5, axis);
    std::get<Eigen::Vector3d>(a) << 1, 2, 3;
    std::get<Eigen::Vector3d>(b) << 1, 0, -1;

    c = interpolate(space, a, b, 0.1);

    EXPECT(std::get<Eigen::Vector3d>(c)[0]) == 1.0;
    EXPECT(std::get<Eigen::Vector3d>(c)[1]) == 1.8;
    EXPECT(std::get<Eigen::Vector3d>(c)[2]) == 2.6;


    Eigen::AngleAxisd aa;
    aa = std::get<Eigen::Quaterniond>(c);

    EXPECT((aa.axis() - axis).squaredNorm()) < 1e-15;
    EXPECT(std::abs(aa.angle() - (0.5 + 2*0.1))) < 1e-10;
}
