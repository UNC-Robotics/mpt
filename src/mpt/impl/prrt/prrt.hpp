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

#pragma once
#ifndef MPT_IMPL_PRRT_PLANNER_HPP
#define MPT_IMPL_PRRT_PLANNER_HPP

#include "edge.hpp"
#include "node.hpp"
#include "../atom.hpp"
#include "../goal_has_sampler.hpp"
#include "../link_trajectory.hpp"
#include "../object_pool.hpp"
#include "../planner_base.hpp"
#include "../scenario_goal.hpp"
#include "../scenario_link.hpp"
#include "../scenario_rng.hpp"
#include "../scenario_sampler.hpp"
#include "../scenario_space.hpp"
#include "../timer_stat.hpp"
#include "../worker_pool.hpp"
#include "../../log.hpp"
#include "../../random_device_seed.hpp"
#include <forward_list>
#include <mutex>
#include <utility>

namespace unc::robotics::mpt::impl::prrt {

    template <bool enable>
    struct WorkerStats;

    template <>
    struct WorkerStats<false> {
        void countIteration() const {}
        void countBiasedSample() const {}
        auto& validMotion() { return TimerStat<void>::instance(); }
        auto& nearest() { return TimerStat<void>::instance(); }
    };

    template <>
    struct WorkerStats<true> {
        mutable std::size_t iterations_{0};
        mutable std::size_t biasedSamples_{0};
        mutable TimerStat<> validMotion_;
        mutable TimerStat<> nearest_;

        void countIteration() const { ++iterations_; }
        void countBiasedSample() const { ++biasedSamples_; }

        TimerStat<>& validMotion() const { return validMotion_; }
        TimerStat<>& nearest() const { return nearest_; }

        WorkerStats& operator += (const WorkerStats& other) {
            iterations_ += other.iterations_;
            biasedSamples_ += other.biasedSamples_;
            validMotion_ += other.validMotion_;
            nearest_ += other.nearest_;
            return *this;
        }

        void print() const {
            MPT_LOG(INFO) << "iterations: " << iterations_;
            MPT_LOG(INFO) << "biased samples: " << biasedSamples_;
            MPT_LOG(INFO) << "valid motion: " << validMotion_;
            MPT_LOG(INFO) << "nearest: " << nearest_;
        }
    };

    template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
    class PRRT : public PlannerBase<PRRT<Scenario, maxThreads, reportStats, NNStrategy>> {
        using Planner = PRRT;
        using Base = PlannerBase<Planner>;
        using Space = scenario_space_t<Scenario>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Link = scenario_link_t<Scenario>;
        using Traj = link_trajectory_t<Link>;
        using Node = prrt::Node<State, Traj>;
        using Edge = prrt::Edge<State, Traj>;
        using RNG = scenario_rng_t<Scenario, Distance>;
        using Sampler = scenario_sampler_t<Scenario, RNG>;

        Distance maxDistance_{std::numeric_limits<Distance>::infinity()};
        Distance goalBias_{0.01};

        static constexpr bool concurrent = maxThreads != 1;
        using NNConcurrency = std::conditional_t<concurrent, nigh::Concurrent, nigh::NoThreadSafety>;
        nigh::Nigh<Node*, Space, NodeKey, NNConcurrency, NNStrategy> nn_;

        std::mutex mutex_;
        std::forward_list<Node*> goals_;

        // we could use goals_.size(), but the performance of check if
        // there is a solution can be critical, and goals_.size()
        // requires a mutex lock.  Thus we store the goal count in an
        // atomic for fast access.
        Atom<std::size_t, concurrent> goalCount_{0};

        ObjectPool<Node, false> startNodes_;

        struct Worker;

        WorkerPool<Worker, maxThreads> workers_;

        void foundGoal(Node* node) {
            MPT_LOG(INFO) << "found solution";
            {
                std::lock_guard<std::mutex> lock(mutex_);
                goals_.push_front(node);
            }
            ++goalCount_;
        }

    public:
        template <typename RNGSeed = RandomDeviceSeed<>>
        explicit PRRT(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
            : nn_(scenario.space())
            , workers_(scenario, seed)
        {
            MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
            MPT_LOG(TRACE) << "Using concurrency: " << log::type_name<NNConcurrency>();
            MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();
        }

        void setGoalBias(Distance bias) {
            assert(0 <= bias && bias <= 1);
            goalBias_ = bias;
        }

        Distance getGoalBias() const {
            return goalBias_;
        }

        void setRange(Distance range) {
            assert(range > 0);
            maxDistance_ = range;
        }

        Distance getRange() const {
            return maxDistance_;
        }

        std::size_t size() const {
            return nn_.size();
        }

        template <typename ... Args>
        void addStart(Args&& ... args) {
            std::lock_guard<std::mutex> lock(mutex_);
            Node *node = startNodes_.allocate(Traj{}, nullptr, std::forward<Args>(args)...);
            // TODO: workers_[0].connect(node);
            nn_.insert(node);
        }

        // required to get convenience methods
        using Base::solveFor;
        using Base::solveUntil;

        // required method
        template <typename DoneFn>
        std::enable_if_t<std::is_same_v<bool, std::result_of_t<DoneFn()>>>
        solve(DoneFn doneFn) {
            if (size() == 0)
                throw std::runtime_error("there are no valid initial states");

            workers_.solve(*this, doneFn);
        }

        bool solved() const {
            return goalCount_.load(std::memory_order_relaxed);
        }

    private:
        std::pair<Distance, std::size_t> pathCost(const Node *n) const {
            Distance cost = 0;
            std::size_t size = 0;
            if (n) {
                ++size;
                for (const Node *p ; (p = n->parent()) != nullptr ; n = p) {
                    cost += workers_[0].space().distance(n->state(), p->state());
                    ++size;
                }
            }
            return {cost, size};
        }

        std::pair<const Node*, std::size_t> bestSolution() const {
            Distance bestCost = std::numeric_limits<Distance>::infinity();
            std::size_t bestSize = 0;
            const Node* bestGoal = nullptr;
            for (const Node *goal : goals_) {
                auto [cost, size] = pathCost(goal);
                if (cost < bestCost || bestGoal == nullptr) {
                    bestCost = cost;
                    bestSize = size;
                    bestGoal = goal;
                }
            }
            return {bestGoal, bestSize};
        }
        
        template <typename Fn>
        std::enable_if_t< is_trajectory_callback_v< Fn, State, Traj> >
        solutionRecur(const Node *node, Fn& fn) const {
            if (const Node *p = node->parent()) {
                solutionRecur(p, fn);
                fn(p->state(), node->edge().link(), node->state(), true);
            }
        }

        template <typename Fn>
        std::enable_if_t< is_trajectory_reference_callback_v< Fn, State, Traj > >
        solutionRecur(const Node *node, Fn& fn) const {
            if (const Node *p = node->parent()) {
                solutionRecur(p, fn);
                fn(p->state(), *node->edge().link(), node->state(), true);
            }
        }

        template <typename Fn>
        std::enable_if_t< is_waypoint_callback_v< Fn, State, Traj > >
        solutionRecur(const Node *node, Fn& fn) const {
            if (const Node *p = node->parent())
                solutionRecur(p, fn);
            fn(node->state());
        }

    public:
        std::vector<State> solution() const {
            auto [n, size] = bestSolution();
            std::vector<State> path;
            if (n) {
                path.reserve(size);
                do {
                    path.push_back(n->state());
                } while ((n = n->parent()) != nullptr);
                std::reverse(path.begin(), path.end());
            }
            return path;
        }

        template <typename Fn>
        void solution(Fn fn) const {
            auto [goal, size] = bestSolution();
            // Either call:
            // fn(n)             size times
            // or
            // fn(n, traj, n)   (size-1) times
            if (goal)
                solutionRecur(goal, fn);
        }

        void printStats() const {
            MPT_LOG(INFO) << "nodes in graph: " << nn_.size();
            auto [cost, size] = bestSolution();
            MPT_LOG(INFO) << "solutions: " << goalCount_.load() << ", best cost=" << cost
                          << " over " << size << " waypoints";
            if constexpr (reportStats) {
                WorkerStats<true> stats;
                for (unsigned i=0 ; i<workers_.size() ; ++i)
                    stats += workers_[i];
                stats.print();
            }
        }

    private:
        template <typename Visitor, typename Nodes>
        void visitNodes(Visitor&& visitor, const Nodes& nodes) const {
            for (const Node& n : nodes) {
                visitor.vertex(n.state());
                if (n.parent())
                    visitor.edge(n.parent()->state());
            }
        }

    public:
        template <typename Visitor>
        void visitGraph(Visitor&& visitor) const {
            visitNodes(std::forward<Visitor>(visitor), startNodes_);
            for (const Worker& w : workers_)
                visitNodes(std::forward<Visitor>(visitor), w.nodes());
        }
    };

    template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
    class PRRT<Scenario, maxThreads, reportStats, NNStrategy>::Worker
        : public WorkerStats<reportStats>
    {
        using Stats = WorkerStats<reportStats>;

        unsigned no_;
        Scenario scenario_;
        RNG rng_;

        ObjectPool<Node> nodePool_;

    public:
        Worker(Worker&& other)
            : no_(other.no_)
            , scenario_(other.scenario_)
            , rng_(other.rng_)
            , nodePool_(std::move(other.nodePool_))
        {
        }

        template <typename RNGSeed>
        Worker(unsigned no, const Scenario& scenario, const RNGSeed& seed)
            : no_(no)
            , scenario_(scenario)
            , rng_(seed)
        {
        }

        // decltype(auto) to allow both 'Space' and 'const Space&'
        // return types.
        decltype(auto) space() const {
            return scenario_.space();
        }

        const auto& nodes() const {
            return nodePool_;
        }

        template <typename DoneFn>
        void solve(Planner& planner, DoneFn done) {
            MPT_LOG(TRACE) << "worker running";

            Sampler sampler(scenario_);
            using Goal = scenario_goal_t<Scenario>;
            if constexpr (goal_has_sampler_v<Goal>) {
                if (no_ == 0 && planner.goalBias_ > 0) {
                    GoalSampler<Goal> goalSampler(scenario_.goal());
                    std::uniform_real_distribution<Distance> uniform01;

                    // since we only have 1 thread performing goal
                    // biased sampling, we scale its percentage by the
                    // number of concurrent threads.
                    Distance scaledBias = planner.goalBias_ * planner.workers_.size();

                    MPT_LOG(TRACE) << "using scaled goal bias of " << scaledBias;

                    while (!done()) {
                        Stats::countIteration();
                        if (planner.goalCount_.load(std::memory_order_relaxed) >= 1)
                            goto unbiasedSamplingLoop;
                        if (uniform01(rng_) < planner.goalBias_) {
                            Stats::countBiasedSample();
                            addSample(planner, goalSampler(rng_));
                        } else {
                            addSample(planner, sampler(rng_));
                        }
                    }
                    return;
                }
            }

          unbiasedSamplingLoop:
            while (!done()) {
                Stats::countIteration();
                addSample(planner, sampler(rng_));
            }

            MPT_LOG(TRACE) << "worker done";
        }

        void addSample(Planner& planner, std::optional<State>&& sample) {
            if (sample)
                addSample(planner, *sample);
        }

        decltype(auto) nearest(Planner& planner, const State& state) {
            Timer timer(Stats::nearest());
            return planner.nn_.nearest(state);
        }

        void addSample(Planner& planner, State& randState) {
            // nearest returns an optional, however it will
            // only be empty if the nn structure is empty,
            // which it will not be, because the planner's
            // solve checks first. (but we assert anyways)
            auto [nearNode, d] = nearest(planner, randState).value();

            State newState = randState;

            // avoid adding the same state multiple times.  Unfortunately
            // this check is not sufficient, and may need to be updated.
            // numeric issues may cause distance() to return a non-zero
            // value when given the same state as both arguments.  It is
            // also possible that non-equivalent states have a zero
            // distance--but this would cause other issues with the
            // planner and thus may not be worth handling.
            if (d == 0)
                return;

            if (d > planner.maxDistance_)
                newState = interpolate(
                    scenario_.space(),
                    nearNode->state(), randState,
                    planner.maxDistance_ / d);

            // TODO: do not need to check when scenario returns
            // std::optional<State> and the motion was not
            // interpolated.
            if (!scenario_.valid(newState))
                return;

            if (auto traj = validMotion(nearNode->state(), newState)) {
                auto [isGoal, goalDist] = scenario_.goal()(scenario_.space(), newState);
                (void)goalDist; // mark unused (for now, may be used in approx solutions)

                Node* newNode = nodePool_.allocate(linkTrajectory(traj), nearNode, newState);
                planner.nn_.insert(newNode);

                if (isGoal)
                    planner.foundGoal(newNode);
            }
        }

        decltype(auto) validMotion(const State& a, const State& b) {
            Timer timer(Stats::validMotion());
            return scenario_.link(a, b);
        }
    };
}

#endif
