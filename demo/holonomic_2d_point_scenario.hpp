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


#ifndef HOLONOMIC_2D_POINT_HPP
#define HOLONOMIC_2D_POINT_HPP

#include <mpt/lp_space.hpp>
#include <mpt/box_bounds.hpp>
#include <mpt/goal_state.hpp>
#include "shape_hierarchy.hpp"
#include <Eigen/Dense>
#include <vector>

namespace mpt_demo {
    using namespace unc::robotics;
    using namespace shape;

    template <typename Scalar = double>
    class Holonomic2DPointScenario {
    public:
        using Space = unc::robotics::mpt::L2Space<Scalar, 2>;
        using Bounds = unc::robotics::mpt::BoxBounds<Scalar, 2>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Goal = unc::robotics::mpt::GoalState<Space>;

        State startState() {
            State q;
            q.fill(10); 
            return q;
        }

        State goalState() {
            State q;
            q << (Scalar) width_ - 10, (Scalar) height_ - 10;
            return q;
        }

    private:
        const int width_; 
        const int height_; 
        Space space_;
        Bounds bounds_;
        Goal goal_;
        const std::vector<Circle<Scalar>> circles_;
        const std::vector<Rect<Scalar>> rects_;

        Bounds makeBounds() {
            Eigen::Matrix<Scalar, 2, 1> min, max;
            min.fill(0);
            max << width_, height_;
            return Bounds(min, max);
        }

    public:
        Holonomic2DPointScenario(
            std::vector<Circle<Scalar>> &circles, 
            std::vector<Rect<Scalar>> &rects,
            int width = 512, 
            int height = 512)
            : width_(width),
              height_(height),
              bounds_(makeBounds()),
              goal_(1e-6, goalState()),
              circles_(circles),
              rects_(rects)
        {            
        }

        bool valid(const State &q) const {
            for(auto const &c : circles_)
                if(!c.pointIsValid(q))
                    return false;
            for(auto const &r : rects_)
                if(!r.pointIsValid(q))
                    return false;
            return true;
        }

        bool link(const State &a, const State &b) const {
            for(auto const &c : circles_)
                if(!c.segmentIsValid(a, b))
                    return false;
            for(auto const &r : rects_)
                if(!r.segmentIsValid(a, b))
                    return false;
            return true;
        }

        const std::vector<Circle<Scalar>> &circles() const{
            return circles_;            
        }

        const Space &space() const {
            return space_;
        }

        const Bounds &bounds() const {
            return bounds_;
        }

        const Goal &goal() const {
            return goal_;
        }
    };
}
#endif