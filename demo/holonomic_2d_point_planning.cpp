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
#include <mpt/prrt.hpp>
#include <fstream>
#include <Eigen/Dense>

using namespace unc::robotics::mpt;
using namespace mpt_demo;
using namespace std::literals;
using namespace shape;

using Scalar = double;
using Algorithm = PRRTStar<>;
using Scenario = Holonomic2DPointScenario<Scalar>;
using State = Scenario::State;


int main(int argc, char *argv[])
{
    /*
     *  1.  Set the canvas size and add obstacles.
     *
     *      There are two obstacles classes: Circles with parameters (center_x, center_y, radius)
     *      and Rect<Scalar> with parameters (x1, y1, x2, y2) where (x1, y1) is the coordinate of
     *      the top left corner and (x2, y2) is the coordinate of the bottom right corner.
     *      obstacleColor is an optional parameter. It will be used to fill the obstacles in the svg output.
     */
    const int width = 1024;
    const int height = 512;

    std::vector<Circle<Scalar>> circles;
    std::vector<Rect<Scalar>> rects;

    Color obstacleColor(70, 70, 70);
    circles.push_back(Circle<Scalar>(170.0, 140.0, 80.0, obstacleColor));
    circles.push_back(Circle<Scalar>(800.0, 70.0, 50.0, obstacleColor));
    circles.push_back(Circle<Scalar>(900.0, 380.0, 70.0, obstacleColor));
    rects.push_back(Rect<Scalar>(375, 140, 520, 220, obstacleColor));
    rects.push_back(Rect<Scalar>(200, 320, 390, 390, obstacleColor));
    rects.push_back(Rect<Scalar>(600, 200, 680, 450, obstacleColor));

    /*
     *  3.  Set the start state and the goal state, then initialize scenario.
     */

    State startState, goalState;
    startState << 30, 30;
    goalState << width - 30, height - 30;
    Scenario scenario(width, height, circles, rects, goalState);

    /*
     *  4.  Initialize and run the planner.
     *
     *      We initialize the planner with the scenario above. Then, we add the
     *      start state and run the planner for MAX_SOLVE_TIME (milliseconds).
     *      Planner will use the specified algorithm to explore possible paths
     *      from the start to the goal state in the given scenario.
     */
    static constexpr auto MAX_SOLVE_TIME = 50ms;
    Planner<Scenario, Algorithm> planner(scenario);
    planner.addStart(startState);
    planner.solveFor(MAX_SOLVE_TIME);
    planner.printStats();
    std::vector<State> solution = planner.solution();

    /*
     *  5.  Output the result to a svg file.
     *
     *      svg export functions are defined in shape_hierarchy.hpp.
     */
    const std::string filename = "holonomic_demo_output.svg";
    MPT_LOG(INFO) << "Writing to " << filename;
    std::ofstream file(filename);
    startSvg(file, width, height);

    // add the obstacles to the svg file.
    for (auto &circle : circles)
        file << circle;
    for (auto &rect : rects)
        file << rect;

    // Visitor will be used to traverse the explored paths.
    //
    // It must supply two functions, vertex and edge, that will be used by the planner. Vertex(q) should set
    // the current node (from_) to q. Then, the planner will call edge(to) for each edge in the visited graph.
    // In this case, edge(to) will add each edge(from_, to) to the svg file.
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

    // Next, we traverse and draw the solution the path.
    if (!solution.empty())
    {
        for (auto it = solution.begin(); it + 1 != solution.end() ; ++it)
        {
            const auto &from = *it;
            const auto &to = *(it + 1);
            addSolutionEdge(file, from[0], from[1], to[0], to[1], 3);
        }
    }
    else
    {
        MPT_LOG(INFO) <<  "No solution was found";
    }
    // Finally, we add the start and the goal state. 
    addStartState(file, startState[0], startState[1]);
    addGoalState(file, goalState[0], goalState[1]);
    endSvg(file);

    return 0;
}
