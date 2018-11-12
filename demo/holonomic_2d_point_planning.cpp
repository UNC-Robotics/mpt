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


#include "holonomic_2d_point_scenario.hpp"
#include "shape_hierarchy.hpp"
#include <vector>
#include <cstdio>
#include <mpt/prrt_star.hpp>
#include <fstream>
#include <Eigen/Dense>


int main(int argc, char *argv[])
{
    using namespace unc::robotics::mpt;
    using namespace mpt_demo;
    using namespace std::literals;
    using namespace shape;

    using Scalar = double;
    using Algorithm = PRRTStar<>;
    using Scenario = Holonomic2DPointScenario<Scalar>;
    using State = Scenario::State;

    Color obstacleColor(140, 200, 200);

    std::vector<Circle<Scalar>> circles;
    std::vector<Rect<Scalar>> rects;

    circles.push_back(Circle<Scalar>(170.0, 140.0, 80.0, obstacleColor));
    circles.push_back(Circle<Scalar>(800.0, 70.0, 50.0, obstacleColor));
    circles.push_back(Circle<Scalar>(900.0, 380.0, 70.0, obstacleColor));

    rects.push_back(Rect<Scalar>(375, 140, 520, 220, obstacleColor));
    rects.push_back(Rect<Scalar>(200, 320, 390, 390, obstacleColor));
    rects.push_back(Rect<Scalar>(600, 200, 680, 450, obstacleColor));


    const int width = 1024;
    const int height = 512;

    Scenario scenario(circles, rects, width, height);

    static constexpr auto MAX_SOLVE_TIME = 50ms;
    Planner<Scenario, Algorithm> planner(scenario);
    planner.addStart(scenario.startState());
    // planner.solveFor([&] { return planner.solved(); }, MAX_SOLVE_TIME);
    planner.solveFor(MAX_SOLVE_TIME);
    planner.printStats();
    std::vector<State> solution = planner.solution();

    const std::string filename = "holonomic_demo_output.svg";
    MPT_LOG(INFO) << "Writing to " << filename;
    std::ofstream file(filename);
    startSvg(file, width, height);
    for(auto &circle : circles)
    {
        file << circle;
    }

    for(auto &rect : rects)
    {
        file << rect;
    }

    struct Visitor
    {
        std::ofstream &out_;
        State from_;

        Visitor(std::ofstream &out) : out_(out) {}

        void vertex(const State &q)
        {
            from_ = q;
        }

        void edge(const State &to)
        {
            addVisitedEdge(out_, from_[0], from_[1], to[0], to[1], 0.3);
        }
    };
    planner.visitGraph(Visitor(file));

    if (!solution.empty()) {
        for(auto it = solution.begin(); it+1 != solution.end() ; ++it)
        {
            const auto &from = *it;
            const auto &to = *(it + 1);
            addSolutionEdge(file, from[0], from[1], to[0], to[1], 3);
        }
    }
    else{
        MPT_LOG(INFO) <<  "No solution was found";
    }
    endSvg(file);
    return 0;
}