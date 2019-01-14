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

#include "link_manipulator_scenario.hpp"
#include "shape_hierarchy.hpp"
#include <vector>
#include <cstdio>
#include <mpt/prrt_star.hpp>
#include <fstream>
#include <Eigen/Dense>
#include <cstdio>
#include <cmath>

constexpr int dimension = 5; // number of arms

using namespace unc::robotics::mpt;
using namespace mpt_demo;
using namespace std::literals;
using namespace shape;

using Scalar = double;
using Algorithm = PRRTStar<>;
using Scenario = LinkManipulatorScenario<Scalar, dimension>;
using State = Scenario::State;
using Point = Eigen::Matrix<Scalar, 2, 1>;

void drawState(std::ofstream &file, Scalar radius, State &s, std::vector<Scalar> &armLengths, double opacity);
void interpolate(std::ofstream &file, Scalar radius, State &a, State &b, std::vector<Scalar> &armLengths, double opacity);

int main()
{
    std::vector<Scalar> armLengths = {10.0, 12.0, 8.0, 6.0, 4.0};
    Scalar lengthSum = 0.0;
    for (const auto &len : armLengths)
        lengthSum += len;
    const Scalar radius = 0.5;

    std::vector<Circle<Scalar>> circles;
    Color obstacleColor(70, 70, 70);
    circles.push_back(Circle<Scalar>(20, -20, 8, obstacleColor));
    circles.push_back(Circle<Scalar>(-20, -30, 5, obstacleColor));
    circles.push_back(Circle<Scalar>(0, 25, 10, obstacleColor));
    circles.push_back(Circle<Scalar>(30, 10, 10, obstacleColor));
    circles.push_back(Circle<Scalar>(-30, 10, 8, obstacleColor));

    State startState, goalState;
    Scalar pi = PI<Scalar>;
    startState << -pi * 5 / 6, 0, 0, 0, 0;
    goalState.fill(0);
    goalState[0] = PI<Scalar> - (Scalar) 1e-10;
    Scenario scenario(goalState, circles, armLengths, radius);

    static constexpr auto MAX_SOLVE_TIME = 100ms;
    Planner<Scenario, Algorithm> planner(scenario);
    planner.addStart(startState);
    planner.solveFor(MAX_SOLVE_TIME);
    planner.printStats();
    std::vector<State> solution = planner.solution();

    if (solution.empty())
    {
        MPT_LOG(INFO) <<  "No solution was found";
        return 0;
    }

    const std::string filename = "solution.js";
    MPT_LOG(INFO) << "Writing to " << filename;
    std::ofstream file(filename);

    file << "data = " << std::endl;
    file << "{" << std:: endl;

    // TODO: what if no solution is found?

    // 1. radius
    file << "\t\"radius\": " << radius << "," << std::endl << std::endl;

    // 2. dimension
    file << "\t\"dimension\": " << dimension << "," << std::endl << std::endl;

    // 3. armLengths
    file << "\t\"armLengths\": [";
    for (auto it = armLengths.begin(); it != armLengths.end(); ++it)
    {
        const auto &l = *it;
        file << l;
        if (it < armLengths.end() - 1)
            file << ", ";

    }
    file << "]," << std::endl << std::endl;

    // 4. lengthSum
    file << "\t\"lengthSum\": " << lengthSum << "," << std::endl << std::endl;

    // 5. circles
    file << "\t\"circles\": [" << std::endl;
    for (auto it = circles.begin(); it != circles.end(); ++it)
    {
        const auto &c = *it;
        file << "\t\t{" << std::endl;

        file << "\t\t\t\"" << "cx" << "\": " << c.cx() << "," << std::endl;
        file << "\t\t\t\"" << "cy" << "\": " << c.cy() << "," << std::endl;
        file << "\t\t\t\"" << "r" << "\": " << c.r() << "" << std::endl;
        file << std::endl;

        file << "\t\t}";
        if (it < circles.end() - 1)
            file << "," << std::endl;
        file << std::endl;
    }
    file << "\t]," << std::endl << std::endl;

    // 6. Solution
    file << "\t\"solution\": [" << std::endl;
    State from, to;
    for (auto it = solution.begin(); it != solution.end(); ++it)
    {
        const State &s = *it;
        file << "\t\t[";
        for (int i = 0; i < dimension; ++i) {
            file << s[i];
            if (i < dimension - 1) 
                file << ", ";
        }
        file << "]";

        if (it < solution.end() - 1)
            file << "," << std::endl;
        file << std::endl;        
    }
    file << "\t]" << std::endl;

    file << "}" << std::endl;
    return 0;
}

void drawState(std::ofstream &file, Scalar radius, State &s, std::vector<Scalar> &armLengths, double opacity)
{
    Point from(0, 0);
    Point to;
    Scalar angle = 0.0;
    for (int j = 0; j < dimension; j++)
    {
        angle += s[j];
        to = from + armLengths[j] * Point(cos(angle), sin(angle));
        addSolutionEdge(file, from[0], from[1], to[0], to[1], radius * 2, Color(opacity, opacity, opacity));
        from = to;
    }
}
