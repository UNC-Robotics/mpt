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

#include "png_2d_scenario.hpp"
#include "shape_hierarchy.hpp"
#include <vector>
#include <png.h>
#include <mpt/prrt_star.hpp>

using namespace mpt_demo;
using namespace unc::robotics::mpt;
using namespace mpt_demo;
using namespace std::literals;
using namespace shape;

using Scalar = double;
using State = Eigen::Matrix<Scalar, 2, 1>;
using Algorithm = PRRTStar<>;
using Scenario = PNG2dScenario<Scalar>;

// draws the result to a svg file.
void writeSvgFile(Planner<Scenario, Algorithm>&, const std::string&, int, int, State&, State&);

int main(int argc, char *argv[])
{
    std::string inputName = "../../png_planning_input.png";

    /*
     *  1.  Choose the obstacle colors to be filtered.
     *
     *      FilterColor takes four parameters: r, g, b, and tolerance. It will be used
     *      to filter each pixel that is within the tolerance of the given rgb color.
     */
    std::vector<FilterColor> filters;
    filters.push_back(FilterColor(126, 106, 61, 15));
    filters.push_back(FilterColor(61, 53, 6, 15));
    filters.push_back(FilterColor(255, 255, 255, 5));

    /*
     *  2.  Read and filter the input image.
     *
     *      boolean vector "obstacles" will be used to track the obstacles.
     *      For each pixel (x, y): if obstacles(y * width + x) is true, then 
     *      the pixel is an obstacle. Otherwise, it is not an obstacle.
     */
    //std::vector<bool> obstacles; 
    // int width, height; // width and the height of the input file.
    auto [obstacles, width, height] = readAndFilterPng(filters, inputName); 

    /*
     *  3.  Set the start state and the goal state, then initialize scenario.  
     */
    State startState, goalState;
    startState << 430, 1300; 
    goalState << 3150, 950; 

    Scenario scenario(width, height, goalState, obstacles);

    /*
     *  4.  Initialize and run the planner.
     *
     *      We initialize the planner with the scenario above. Then, we add the 
     *      start state and run the planner for MAX_SOLVE_TIME (milliseconds).  
     *      Planner will use the specified algorithm to explore possible paths
     *      from the start to the goal state in the given scenario.
     */
    static constexpr auto MAX_SOLVE_TIME = 50ms; // maximum runtime allotted for the planner.
    Planner<Scenario, Algorithm> planner(scenario);
    planner.addStart(startState);  
    planner.solveFor(MAX_SOLVE_TIME);
    planner.printStats();

    writeSvgFile(planner, inputName, width, height, startState, goalState); // write the result to svg.

    return 0;
}

inline void writeSvgFile(Planner<Scenario, Algorithm> &planner, const std::string &inputName, int width, int height, State &startState, State &goalState)
{
    std::vector<State> solution = planner.solution();
    if (solution.empty())
    {
        MPT_LOG(INFO) <<  "No solution was found";
        return;
    }

    const std::string outputName = "png_2d_demo.svg";
    std::ofstream file(outputName);
    startSvg(file, width, height);
    addImage(file, inputName);

    MPT_LOG(INFO) << "Writing the solution to " << outputName;

    // draw the visited paths
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
            addVisitedEdge(out_, from_[0], from_[1], to[0], to[1]);
        }
    };
    planner.visitGraph(Visitor(file));

    // draw the solution paths
    if (!solution.empty())
    {
        for (auto it = solution.begin(); it + 1 != solution.end() ; ++it)
        {
            const auto &from = *it;
            const auto &to = *(it + 1);
            addSolutionEdge(file, from[0], from[1], to[0], to[1], 10.0);
        }
    }

    // add the start state and the end state.
    addStartState(file, startState[0], startState[1], 40);
    addGoalState(file, goalState[0], goalState[1], 40);

    endSvg(file);
}
