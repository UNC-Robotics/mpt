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

#ifndef PNG_COLOR_FILTER
#define PNG_COLOR_FILTER

#include <mpt/lp_space.hpp>
#include <mpt/box_bounds.hpp>
#include <mpt/goal_state.hpp>
#include <Eigen/Dense>
#include <iostream> // cout, cerr
#include <fstream> // ifstream
#include <sstream> // stringstream
#include <vector>
#include <png.h>

namespace mpt_demo
{
    struct PNGColor
    {
        PNGColor(int r, int g, int b)
            : r_(r), g_(g), b_(b)
        {
        }
        int r_;
        int g_;
        int b_;

        // TODO: consider the alpha value.
        bool isObstacle(int r, int g, int b, int tol) const
        {
            if((r < r_ - tol || r > r_ + tol) || (g < g_ - tol || g > g_ + tol) || (b < b_ - tol || b > b_ + tol))
            {
                return false;
            }
            return true;
        }
    };

    template <typename Scalar = double>
    class PNG2dScenario
    {
    public:
        using Space = unc::robotics::mpt::L2Space<Scalar, 2>;
        using Bounds = unc::robotics::mpt::BoxBounds<Scalar, 2>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Goal = unc::robotics::mpt::GoalState<Space>;

    private:
        const int width_;
        const int height_;
        Space space_;
        Bounds bounds_;
        Goal goal_;
        png_bytep *rowPointers_;

    public:
        PNG2dScenario(
            const int width,
            const int height,
            State goalState,
            png_bytep *rowPointers
        )
            : width_(width),
              height_(height),
              bounds_(makeBounds()),
              goal_(1e-6, goalState),
              rowPointers_(rowPointers)
        {
        }

        bool valid(const State &q) const
        {
            int x = (int) (q[0] + 0.5);
            int y = (int) (q[1] + 0.5);

            png_bytep px = get(q[0], q[1]);
            if(px[0] == 0 && px[1] == 0 && px[2] == 0)
                return false;
            else
                return true;
        }

        bool link(const State &a, const State &b) const
        {
            if(!valid(a) || !valid(b))
                return false;
            return validSegment(a, b);
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
    private:
        Bounds makeBounds()
        {
            Eigen::Matrix<Scalar, 2, 1> min, max;
            min.fill(0);
            max << width_, height_;
            return Bounds(min, max);
        }

        bool validSegment(const State &a, const State &b) const
        {
            // uses bisection method to verify links.
            State mid = (a + b) / 2;
            Scalar distSquared = (b - a).squaredNorm();
            Scalar tolerance = 0.1;
            if(distSquared < tolerance * tolerance)
                return true;
            if(!valid(mid))
                return false;
            bool left = validSegment(a, mid);
            bool right = validSegment(mid, b);
            return left && right;
        }

        png_bytep get(int x, int y) const
        {
            png_bytep row = rowPointers_[y];
            png_bytep px = &(row[x * 4]);
            return px;
        }
    };
}
#endif