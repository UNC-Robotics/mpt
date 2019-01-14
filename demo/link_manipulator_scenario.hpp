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

//! @author William Lee

#ifndef LINK_MANIPULATOR_SCENARIO
#define LINK_MANIPULATOR_SCENARIO

#include "shape_hierarchy.hpp"
#include <mpt/lp_space.hpp>
#include <mpt/so2_space.hpp>
#include <mpt/box_bounds.hpp>
#include <mpt/goal_state.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace shape;

template <class T> constexpr T PI = T(3.14159265358979323846264338327950288419716939937510582097494459230781640628620L);

namespace mpt_demo
{
    template <typename Scalar, int dimensions>
    class LinkManipulatorScenario
    {
        static_assert(dimensions > 0, "There must be at least one arm");
    public:
        using Space = unc::robotics::mpt::L1Space<Scalar, dimensions>;
        using Bounds = unc::robotics::mpt::BoxBounds<Scalar, dimensions>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Goal = unc::robotics::mpt::GoalState<Space>;
        using Point = Eigen::Matrix<Scalar, 2, 1>;


    private:
        Space space_;
        Bounds bounds_;
        Goal goal_;
        std::vector<Circle<Scalar>> circles_;
        std::vector<Scalar> armLengths_;
        Scalar radius_;

    public:
        LinkManipulatorScenario(
            State goalState,
            std::vector<Circle<Scalar>> &circles,
            std::vector<Scalar> &armLengths,
            Scalar radius
        )
            :
            bounds_(makeBounds()),
            goal_(1e-6, goalState),
            circles_(circles),
            armLengths_(armLengths),
            radius_(radius)
        {
        }

        static Bounds makeBounds()
        {
            State min, max;
            min.fill(-PI<Scalar>);
            max.fill(PI<Scalar>);
            return Bounds(min, max);
        }

        bool valid(const State &q) const
        {
            Point from(0, 0);
            Point to;
            Scalar angle = 0.0;
            for (int i = 0; i < dimensions; i++)
            {
                // compute the two ends of each segment
                angle += q[i];
                to = from + armLengths_[i] * Point(cos(angle), sin(angle));
                // check if the segment(from, to) collides with any circle
                for (const auto &c : circles_)
                    if (!c.segmentIsValid(from, to, radius_))
                        return false;
                from = to;
            }
            return true;
        }

        bool link(const State &a, const State &b) const
        {
            if (!valid(a) || !valid(b))
                return false;
            return bisectLink(a, b);
        }

        bool bisectLink(const State &a, const State &b) const
        {
            State diff = a - b;
            Scalar maxAngleDiff = diff.template lpNorm<Eigen::Infinity>();
            constexpr Scalar tolerance = 0.02; // 0.02 rad ~= 1 degree
            if (maxAngleDiff < tolerance)
                return true;
            State mid = (a + b) / 2;
            if (!valid(mid))
                return false;
            if (!bisectLink(a, mid))
                return false;
            return bisectLink(mid, b);
        }

        const Space &space() const
        {
            return space_;
        }

        const Bounds &bounds() const
        {
            return bounds_;
        }

        const Goal &goal() const
        {
            return goal_;
        }

    };
}
#endif