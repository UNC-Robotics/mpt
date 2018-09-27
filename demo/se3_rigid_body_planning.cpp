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

#include "se3_rigid_body_scenario.hpp"
#include "scenario_config.hpp"
#include "proc_info.hpp"
#include <mpt/pprm.hpp>
#include <mpt/prrt.hpp>
#include <mpt/prrt_star.hpp>
#include <nigh/gnat.hpp>
#include <getopt.h>
#include <iomanip>
#include <chrono>

// To compile:
// cd ~/projects/mpt/demo ; clang++ -fopenmp -std=c++17 -I../src -I../../nigh/src -I/usr/local/include -I/usr/include/eigen3 -o se3_rigid_body_planning se3_rigid_body_planning.cpp -lfcl -lassimp

// namespace mpt_demo {
//     template <typename Planner, typename Scenario, typename ... Args>
//     void solve(const Scenario& scenario, Args&& args) {
//         Planner planner(scenario, std::forward<Args>(args)...);
//         planner.solve(2000ms);
//     }
// }

enum PlanningAlgorithm {
    kRRTStarAlgorithm,
    kRRTAlgorithm,
    kPRMAlgorithm,
};

struct Options {
    int solveTimeMillis_{-1};
    int nodeCount_{-1};
    bool terminateWhenSolved_{false};
    PlanningAlgorithm algorithm_{kRRTStarAlgorithm};

    enum {
        kScalarFloat,
        kScalarDouble,
        // long double isn't fully supported by FCL
        //kScalarLongDouble
    } scalarType_{kScalarDouble};

    std::string path_;
    std::string configFile_;

    Options(int argc, char *argv[]) {
        static struct option options[] = {
            { "solve-time", required_argument, 0, 't' },
            { "solved", no_argument, 0, 'S' },
            { "nodes", required_argument, 0, 'n' },
            { "algorithm", required_argument, 0, 'a' },
            { "scalar", required_argument, 0, 's' },
            { nullptr, 0, nullptr, 0}
        };

        for (int c, optInd ; -1 != (c=getopt_long(argc, argv, "t:s:n:a:S", options, &optInd)) ; ) {
            std::size_t pos;
            std::string arg;

            switch (c) {
            case 't':
                arg = optarg;
                solveTimeMillis_ = std::stoi(arg, &pos);
                if (pos != arg.length() || solveTimeMillis_ < 0)
                    throw std::invalid_argument("invalid solve time: " + arg);
                break;
            case 'n':
                arg = optarg;
                nodeCount_ = std::stoi(arg, &pos);
                if (pos != arg.length() || nodeCount_ < 0)
                    throw std::invalid_argument("invalid node count: " + arg);
                break;
            case 'S':
                terminateWhenSolved_ = true;
                break;
            case 'a':
                if (std::strcmp("rrtstar", optarg) == 0) {
                    algorithm_ = kRRTStarAlgorithm;
                } else if (std::strcmp("rrt", optarg) == 0) {
                    algorithm_ = kRRTAlgorithm;
                } else if (std::strcmp("prm", optarg) == 0) {
                    algorithm_ = kPRMAlgorithm;
                } else {
                    throw std::invalid_argument("expected algorithm to be 'rrtstar' or 'rrt'");
                }
                break;
            case 's':
                if (std::strcmp("float", optarg) == 0) {
                    scalarType_ = kScalarFloat;
                } else if (std::strcmp("double", optarg) == 0) {
                    scalarType_ = kScalarDouble;
                // } else if (std::strcmp("long_double", optarg) == 0) {
                //     scalarType_ = kScalarLongDouble;
                } else {
                    throw std::invalid_argument("expected scalar type of 'float' or 'double'"); // , or 'long_double'");
                }
                break;
            default:
                std::cerr << "usage: " << argv[0] << " [options] config-file.cfg\n"
                    "Options:\n"
                    "  -t --solve-time=TIME   Specify the time in milliseconds to spend solving\n"
                    "  -S --solved            Run until solved\n"
                    "  -n --nodes=N           Run until planner has generated N\n"
                    "  -a --algorithm=ALG     Run the planning algorithm (rrt or rrtstar)\n"
                    "  -t TIME\n"
                          << std::flush;
                throw std::invalid_argument("unrecognized option");
            }
        }

        if (optind+1 != argc)
            throw std::invalid_argument("expected configuration file");

        configFile_ = argv[optind];

        std::size_t lastSlash = configFile_.find_last_of("\\/");
        path_ = (lastSlash == std::string::npos) ? "" : configFile_.substr(0, lastSlash+1);
    }
};

template <typename Scenario, typename Algorithm, typename Config, typename Scalar>
void runPlanner(
    const Options& options,
    const mpt_demo::ScenarioConfig<>& config,
    const std::string& envMesh, const std::vector<std::string>& robotMeshes,
    const Config& qStart,
    const Config& qGoal,
    const Eigen::Matrix<Scalar, 3, 1>& volumeMin,
    const Eigen::Matrix<Scalar, 3, 1>& volumeMax)
{
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::mpt;
    using namespace std::literals;

    MPT_LOG(INFO) << "Algorithm: " << log::type_name<Algorithm>();
    using Clock = std::chrono::steady_clock;

    Scenario scenario(envMesh, robotMeshes, qGoal, volumeMin, volumeMax, 0.01);

    Planner<Scenario, Algorithm> planner(scenario);
    // planner.addGoal(qGoal);
    planner.addStart(qStart);

    // TODO: only do this for Algorithms that have setRange
    if (config.hasProp("planner", "rrt.range")) {
        Scalar range;
        config.load(range, "planner", "rrt.range");
        MPT_LOG(INFO) << "setting range: " << range;
        planner.setRange(range);
    }

    Clock::time_point start;
    if (options.solveTimeMillis_ > 0) {
        MPT_LOG(INFO) << "solving for " << options.solveTimeMillis_ << " ms";
        start = Clock::now();
        planner.solveFor(std::chrono::milliseconds(options.solveTimeMillis_));
    } else if (options.nodeCount_ > 0) {
        MPT_LOG(INFO) << "solving to node count " << options.nodeCount_;
        start = Clock::now();
        planner.solve([&, count = (std::size_t)options.nodeCount_] () { return planner.size() >= count; });
    } else if (options.terminateWhenSolved_) {
        MPT_LOG(INFO) << "solving until solution";
        start = Clock::now();
        planner.solve([&] () { return planner.solved(); });
    }
    Clock::duration elapsed = Clock::now() - start;
    // TODO: planner.logStats();

    MPT_LOG(INFO) << "planner generated " << planner.size() << " states";
    MPT_LOG(INFO) << "solve time " << std::chrono::duration<double>(elapsed).count() << " seconds";

    planner.printStats();

    if (planner.solved()) {
        std::vector<Config> path = planner.solution();

        Scalar cost = 0;
        auto curr = path.begin();
        if (curr != path.end()) {
            for (auto prev = curr ; ++curr != path.end() ; prev = curr)
                cost += scenario.space().distance(*prev, *curr);
        }
        MPT_LOG(INFO) << "path cost " << cost;

        if (false) {
            for (const Config& q : path) {
                std::cout << std::setprecision(std::numeric_limits<Scalar>::digits10 + 1)
                          << std::get<1>(q)[0] << ' '
                          << std::get<1>(q)[1] << ' '
                          << std::get<1>(q)[2] << ' '
                          << std::get<0>(q).coeffs()[0] << ' '
                          << std::get<0>(q).coeffs()[1] << ' '
                          << std::get<0>(q).coeffs()[2] << ' '
                          << std::get<0>(q).coeffs()[3] << '\n';
            }
        }
    }

    std::cout << mpt_demo::ProcInfo() << std::flush;
}

template <typename Scalar>
void solve(const Options& options, const mpt_demo::ScenarioConfig<>& config) {
    using namespace unc::robotics::mpt;
    using namespace unc::robotics::nigh;

    using Scenario = mpt_demo::SE3RigidBodyScenario<Scalar>;
    using Space = typename Scenario::Space;
    using Config = typename Space::Type;

    Config qGoal;
    Config qStart;

    config.load(qGoal, "problem", "goal");
    config.load(qStart, "problem", "start");

    MPT_LOG(INFO) << "start: " << qStart;
    MPT_LOG(INFO) << "goal: " << qGoal;

    std::string envMesh;
    config.load(envMesh, "problem", "world");
    envMesh = options.path_ + envMesh;

    std::string robotMesh;
    config.load(robotMesh, "problem", "robot");
    robotMesh = options.path_ + robotMesh;

    Eigen::Matrix<Scalar, 3, 1> volumeMin;
    Eigen::Matrix<Scalar, 3, 1> volumeMax;
    config.load(volumeMin, "problem", "volume.min");
    config.load(volumeMax, "problem", "volume.max");

    using NN = NN_TYPE<>;

#if MT
    using Threads = hardware_concurrency;
#else
    using Threads = single_threaded;
#endif

    std::vector<std::string> robotMeshes{{robotMesh}};
    if (options.algorithm_ == kRRTStarAlgorithm) {
        using Algorithm = PRRTStar<report_stats<true>, NN, Threads>;
        runPlanner<Scenario, Algorithm>(options, config, envMesh, robotMeshes, qStart, qGoal, volumeMin, volumeMax);
    } else if (options.algorithm_ == kRRTAlgorithm) {
        using Algorithm = PRRT<report_stats<true>, NN, Threads>;
        runPlanner<Scenario, Algorithm>(options, config, envMesh, robotMeshes, qStart, qGoal, volumeMin, volumeMax);
    } else if (options.algorithm_ == kPRMAlgorithm) {
        // using Algorithm = PPRM<report_stats<true>, NN, Threads>;
        // runPlanner<Scenario, Algorithm>(options, config, envMesh, robotMeshes, qStart, qGoal, volumeMin, volumeMax);
    }
}


int main(int argc, char *argv[]) {
    using namespace mpt_demo;

    try {
        Options options(argc, argv);

        ScenarioConfig<> config(options.configFile_);

        using Scalar = SCALAR_TYPE;

        solve<Scalar>(options, config);

        // switch (options.scalarType_) {
        // case Options::kScalarFloat:
        //     solve<float>(options, config);
        //     break;
        // case Options::kScalarDouble:
        //     solve<double>(options, config);
        //     break;
        // // case Options::kScalarLongDouble:
        // //     solve<long double>(options, config);
        // //     break;
        // }
        return 0;
    } catch (const std::exception& ex) {
        MPT_LOG(FATAL) << "terminated with exception: " << ex.what();
        return 1;
    }
}
