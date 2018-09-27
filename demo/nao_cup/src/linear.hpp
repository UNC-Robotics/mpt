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

#ifndef LINEAR_H
#define LINEAR_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace nao_cup {
    template <typename S>
    using Transform = Eigen::Transform<S, 3, Eigen::Isometry>;

    template <typename S>
    using Vec2 = Eigen::Matrix<S, 2, 1>;
    template <typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;
    template <typename S>
    using Vec4 = Eigen::Matrix<S, 4, 1>;

    template <typename S>
    S v3_len(const Vec3<S> *v) { return v->norm(); }

    template <typename S>
    S v3_dot(const Vec3<S> *a, const Vec3<S> *b) {
        return a->dot(*b);
    }

    template <typename S>
    S v3_dist(const Vec3<S> *a, const Vec3<S> *b) {
        return (*a - *b).norm();
    }

    template <typename S>
    void v3_sub(Vec3<S>* r, const Vec3<S>* a, const Vec3<S>* b) { *r = *a - *b; }

    template <typename S>
    void v3_add(Vec3<S>* r, const Vec3<S>* a, const Vec3<S>* b) { *r = *a + *b; }

    template <typename S>
    void v3_scale(Vec3<S>* r, const Vec3<S> *v, S s) { *r = *v * s; }


    template <typename S>
    void m4_mul(
        Transform<S>* r,
        const Transform<S> *a,
        const Transform<S> *b)
    {
        *r = *a * *b;
    }

    template <typename S>
    void m4_rotate(
        Transform<S> *m,
        const Transform<S> *t,
        S a, S x, S y, S z)
    {
        *m = *t * Eigen::AngleAxis<S>(a, Vec3<S>(x, y, z));
    }

    template <typename S>
    void m4_translate(Transform<S> *m, const Transform<S> *t, S x, S y, S z) {
        *m = *t * Eigen::Translation<S, 3>(x, y, z);
    }

    template <typename S>
    void m4_transform_i3(Vec3<S> *r, const Transform<S> *m, S x, S y, S z) {
        *r = *m * Vec3<S>(x, y, z);
    }

    template <typename S>
    void m4_extract_translation(
        Vec3<S>* r,
        const Transform<S> *m)
    {
        *r = m->translation().template head<3>();
    }

    template <typename S>
    void m4_transform_i(Vec4<S> *r, const Transform<S> *m, S x, S y, S z, S w) {
        *r = *m * Vec4<S>(x, y, z, w);
    }

    template <typename S>
    void m4_transform_i(Vec3<S>* r, const Transform<S> *m, S x, S y, S z) {
        *r = *m * Vec3<S>(x, y, z);
    }

    template <typename S>
    void v3_norm(Vec3<S> *r, const Vec3<S> *a) {
        *r = a->normalized();
    }

    template <typename S>
    S v3_dist_segment_point(const Vec3<S> *s0, const Vec3<S> *s1, const Vec3<S> *pt) {
        Vec3<S> v, w;
        S c1, c2, b;

        v3_sub(&v, s1, s0);
        v3_sub(&w, pt, s0);

        c1 = v3_dot(&w, &v);
        if (c1 <= 0.0) {
            return v3_len(&w);
        }

        c2 = v3_dot(&v, &v);
        if (c2 <= c1) {
            return v3_dist(pt, s1);
        }

        b = c1 / c2;

        v3_scale(&v, &v, b);
        v3_add(&v, s0, &v);

        return v3_dist(&v, pt);
    }
}

#endif /* LINEAR_H */
