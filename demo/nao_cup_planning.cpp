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

#include "nao_cup/src/naocup.hpp"
#include "proc_info.hpp"
#include <mpt/lp_space.hpp>
#include <mpt/box_bounds.hpp>
#include <mpt/prrt.hpp>
#include <mpt/prrt_star.hpp>
#include <mpt/pprm.hpp>
#include <mpt/goal_state.hpp>
#include <memory>
#include <getopt.h>

namespace mpt_demo {
    using namespace unc::robotics::mpt;

    template <typename S>
    class NaoCupScenario {
    public:
        static constexpr int kDimensions =  10;
        using Scalar = S;
        using Space = L2Space<Scalar, kDimensions>;
        using Config = typename Space::Type;

        Space space_;
#if 0
        class Goal {
            mutable void *instance_;

        public:
            Goal(void *instance) : instance_{instance} {
            }

            std::pair<bool, Scalar> operator() (const Space&, const Config& q) const {
                return {nao_in_goal(instance_, q.data()), Scalar(0)};
            }
        };

        Goal goal() const {
            return instance_;
        }
#else
        using Goal = GoalState<Space>;

        const Goal& goal() const {
            static const Goal inst(
                1e-5,
                (Config() <<
                 S(0.258284303377494),
                 S(-0.2699099199363406),
                 S(-0.01113121187052224),
                 S(1.2053012757652763),
                 S(1.2716626717484503),
                 S(-0.9826967097045605),
                 S(0.07355836822937814),
                 S(0.25450053440459897),
                 S(-0.9512909033938429),
                 S(-0.5297424293532234)).finished());
            return inst;
        }
#endif

        std::shared_ptr<unsigned> counter_;
        unsigned no_;
        std::shared_ptr<nao_cup::prrts_system<S>> system_;
        mutable void *instance_;

        NaoCupScenario()
            : counter_{new unsigned(0u)}
            , no_{(*counter_)++}
            , system_(nao_cup::naocup_create_system<S>(), nao_cup::naocup_free_system<S>)
            , instance_(system_->system_data_alloc_func(no_, nullptr, nullptr))
        {
            MPT_LOG(TRACE) << "created scenario " << no_;
        }

        NaoCupScenario(NaoCupScenario&& other)
            : counter_(std::move(other.counter_))
            , no_(other.no_)
            , system_(std::move(other.system_))
            , instance_(std::exchange(other.instance_, nullptr))
        {
            MPT_LOG(TRACE) << "moved scenario " << no_;
        }

        NaoCupScenario(const NaoCupScenario& other)
            : counter_(other.counter_)
            , no_{(*counter_)++}
            , system_(other.system_)
            , instance_(system_->system_data_alloc_func(no_, nullptr, nullptr))
        {
            MPT_LOG(TRACE) << "created scenario " << no_;
        }

        ~NaoCupScenario() {
            MPT_LOG(TRACE) << "free scenario " << no_;
            system_->system_data_free_func(instance_);
        }

        const Space& space() const {
            return space_;
        }

        const BoxBounds<Scalar, kDimensions> bounds() const {
            using Map = Eigen::Map<const Eigen::Matrix<Scalar, kDimensions, 1>>;
            static BoxBounds<Scalar, kDimensions> inst{
                Map(nao_cup::nao_min_config<S>()),
                Map(nao_cup::nao_max_config<S>())
            };
            return inst;
        }

        bool valid(const Config& q) const {
            return nao_cup::nao_clear(instance_, q.data());
        }

        bool link(const Config& a, const Config& b) const {
            return nao_cup::nao_link(instance_, a.data(), b.data());
        }
    };
}

template <typename S, typename Algorithm>
int runPlanner(int solveTimeMillis, int nodeCount, bool terminateWhenSolved) {
    using namespace mpt_demo;
    using namespace unc::robotics::mpt;
    using Scenario = NaoCupScenario<S>;
    using Space = typename Scenario::Space;
    using Config = typename Space::Type;

    Config qGoal;
    qGoal <<
        S(0.258284303377494),
        S(-0.2699099199363406),
        S(-0.01113121187052224),
        S(1.2053012757652763),
        S(1.2716626717484503),
        S(-0.9826967097045605),
        S(0.07355836822937814),
        S(0.25450053440459897),
        S(-0.9512909033938429),
        S(-0.5297424293532234);

    MPT_LOG(INFO) << "state type: " << log::type_name<Config>();
    Scenario scenario;
    auto space = scenario.space();
    bool isGoalV = scenario.goal()(space, qGoal).first;
    MPT_LOG(INFO) << "GOAL: " << isGoalV;
    Planner<Scenario, Algorithm> planner(scenario);

    planner.addStart(Eigen::Map<const Config>(nao_cup::nao_init_config<S>()));
    using Clock = std::chrono::steady_clock;
    //planner.solve([&] { return planner.solved(); });
    //Clock::duration maxSolveTime = 10s;
    // planner.solve([start] { return Clock::now() - start >= maxSolveTime; });
    // planner.setRange(2.5);

    // setGoalBias is not available on all planners
    // planner.setGoalBias(0.01);

    auto start = Clock::now();
    if (solveTimeMillis > 0) {
        if (terminateWhenSolved) {
            planner.solveFor([&] { return planner.solved(); }, std::chrono::milliseconds(solveTimeMillis));
        } else {
            planner.solveFor(std::chrono::milliseconds(solveTimeMillis));
        }
    } else if (nodeCount > 0) {
        planner.solve([&, count = std::size_t(nodeCount)] { return planner.size() >= count; });
    } else if (terminateWhenSolved) {
        planner.solve([&] { return planner.solved(); });
    } else {
        MPT_LOG(ERROR) << "termination condition not specified";
        return 1;
    }

    auto elapsed = Clock::now() - start;
    MPT_LOG(INFO) << "solve time " << elapsed << " seconds";
    planner.printStats();

    if (planner.solved()) {
        std::vector<Config> path = planner.solution();

        S cost = 0;
        auto curr = path.begin();
        if (curr != path.end()) {
            for (auto prev = curr ; ++curr != path.end() ; prev = curr)
                cost += scenario.space().distance(*prev, *curr);
        }
        MPT_LOG(INFO) << "path cost " << cost;
    }

    std::cout << ProcInfo() << std::flush;

    return 0;
}

int main(int argc, char *argv[]) {
    using namespace unc::robotics::mpt;
    using namespace unc::robotics::nigh;
    using namespace mpt_demo;

    static struct option options[] = {
        { "solve-time", required_argument, 0, 't' },
        { "algorithm", required_argument, 0, 'a' },
        { "node-count", required_argument, 0, 'n' },
        { "solved", no_argument, 0, 'S' },
        { nullptr, 0, nullptr, 0 }
    };

    int solveTimeMillis = -1;
    bool terminateWhenSolved = false;
    std::string algorithm = "rrtstar";
    int nodeCount = -1;

    for (int c, optInd ; -1 != (c=getopt_long(argc, argv, "t:a:n:S", options, &optInd)) ; ) {
        std::size_t pos;
        std::string arg;

        switch (c) {
        case 't':
            arg = optarg;
            solveTimeMillis = std::stoi(arg, &pos);
            if (pos != arg.length() || solveTimeMillis < 0)
                throw std::invalid_argument("invalid solve time: " + arg);
            break;
        case 'n':
            arg = optarg;
            nodeCount = std::stoi(arg, &pos);
            if (pos != arg.length() || nodeCount < 0)
                throw std::invalid_argument("invalid node count: " + arg);
            break;
        case 'S':
            terminateWhenSolved = true;
            break;
        case 'a':
            algorithm = optarg;
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " [options]\n"
                "Options:\n"
                "  -t --solve-time=T    Run for T milliseconds\n"
                "  -S --solved          Run until solved\n"
                "  -n --node-count=N    Run until N nodes are generated\n"
                "  -a --algorithm=A     Run algorithm (A = rrt or rrtstar)\n"
                "  -t T\n"
                      << std::flush;
            throw std::invalid_argument("unrecognized option");
        }
    }

    using S = SCALAR_TYPE;
    using NN = NN_TYPE<>;
#if MT
    using Threads = hardware_concurrency;
#else
    using Threads = single_threaded;
#endif

    static constexpr bool reportStats = false;

    if (algorithm == "rrt") {
        using Algorithm = PRRT<report_stats<reportStats>, Threads, NN>;
        return runPlanner<S, Algorithm>(solveTimeMillis, nodeCount, terminateWhenSolved);
    } else if (algorithm == "rrtstar") {
        using Algorithm = PRRTStar<report_stats<reportStats>, Threads, NN>;
        return runPlanner<S, Algorithm>(solveTimeMillis, nodeCount, terminateWhenSolved);
    } else if (algorithm == "prm") {
        using Algorithm = PPRM<report_stats<reportStats>, Threads, NN>;
        return runPlanner<S, Algorithm>(solveTimeMillis, nodeCount, terminateWhenSolved);
    } else {
        MPT_LOG(ERROR) << "algorithm invalid: " << algorithm;
        return 0;
    }
}
