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

#ifndef COLLIDE_H
#define COLLIDE_H

#include "linear.hpp"

namespace nao_cup {

    int collide_trace = 0;

    template <typename S>
    bool collide_sphere_capsule(
        const Transform<S> *sphere_transform, S sphere_radius,
        const Transform<S> *capsule_transform, S capsule_length, S capsule_radius)
    {
        Vec3<S> sphere_center;
        Vec3<S> capsule_start;
        Vec3<S> capsule_end;
        S dist;

        m4_transform_i3<S>(&sphere_center, sphere_transform, S(0), S(0), S(0));
        m4_transform_i3<S>(&capsule_start, capsule_transform, S(0), S(0), S(0));
        m4_transform_i3<S>(&capsule_end, capsule_transform, S(0), S(0), capsule_length);

        // if (collide_trace) {
        //         printf("capsule (%f, %f, %f) to (%f, %f, %f) radius=%f, sphere (%f, %f, %f) radius=%f\n",
        //                capsule_start.x(), capsule_start.y(), capsule_start.z(),
        //                capsule_end.x(), capsule_end.y(), capsule_end.z(), capsule_radius,
        //                sphere_center.x(), sphere_center.y(), sphere_center.z(), sphere_radius);
        // }

        dist = v3_dist_segment_point(&capsule_start, &capsule_end, &sphere_center);

        return dist < (sphere_radius + capsule_radius);
    }

    template <typename S>
    bool collide_sphere_sphere(
        const Transform<S> *a_transform, S a_radius,
        const Transform<S> *b_transform, S b_radius)
    {
        Vec4<S> a_center, b_center;
        S d2, r2;

        m4_transform_i<S>(&a_center, a_transform, S(0), S(0), S(0), S(1));
        m4_transform_i<S>(&b_center, b_transform, S(0), S(0), S(0), S(1));

        d2 = (a_center.template head<3>() - b_center.template head<3>()).squaredNorm();
        r2 = a_radius + b_radius;
        r2 *= r2;

        return (d2 < r2);
    }

    template <typename S>
    bool collide_capsule_capsule(
        const Transform<S> *a_transform, S a_length, S a_radius,
        const Transform<S> *b_transform, S b_length, S b_radius)
    {
        Vec3<S> a_s0, a_s1;
        Vec3<S> b_s0, b_s1;
        S a0, a1;
        S b0, b1;
        S dist;

        m4_transform_i<S>(&a_s0, a_transform, S(0), S(0), S(0));
        m4_transform_i<S>(&a_s1, a_transform, S(0), S(0), a_length);

        m4_transform_i<S>(&b_s0, b_transform, S(0), S(0), S(0));
        m4_transform_i<S>(&b_s1, b_transform, S(0), S(0), b_length);

        a0 = v3_dist_segment_point(&a_s0, &a_s1, &b_s0);
        a1 = v3_dist_segment_point(&a_s0, &a_s1, &b_s1);

        b0 = v3_dist_segment_point(&b_s0, &b_s1, &a_s0);
        b1 = v3_dist_segment_point(&b_s0, &b_s1, &a_s1);

        dist = fmin(fmin(a0, a1), fmin(b0, b1));

        return dist < (a_radius + b_radius);
    }
}

#endif /* COLLIDE_H */
