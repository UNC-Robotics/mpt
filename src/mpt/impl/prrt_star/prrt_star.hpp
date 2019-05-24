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
#ifndef MPT_IMPL_PRRT_STAR_PLANNER_HPP
#define MPT_IMPL_PRRT_STAR_PLANNER_HPP

// TODO: #include <nigh/nigh.hpp>
#include "edge.hpp"
#include "node.hpp"
#include "../atom.hpp"
#include "../constants.hpp"
#include "../goal_has_sampler.hpp"
#include "../link_trajectory.hpp"
#include "../object_pool.hpp"
#include "../planner_base.hpp"
#include "../rrg_rewire_neighbors.hpp"
#include "../scenario_goal.hpp"
#include "../scenario_goal_sampler.hpp"
#include "../scenario_link.hpp"
#include "../scenario_rng.hpp"
#include "../scenario_sampler.hpp"
#include "../scenario_space.hpp"
#include "../timer_stat.hpp"
#include "../worker_pool.hpp"
#include "../../log.hpp"
#include "../../random_device_seed.hpp"
#include <nigh/nigh_forward.hpp>
#include <forward_list>
#include <mutex>
#include <omp.h>
#include <optional>
#include <queue>
#include <stdexcept>

namespace unc::robotics::mpt::impl::prrt_star {

    // // TODO: move this (or something like it to a common place)
    // template <std::intmax_t d, int n = 0>
    // struct log10 : log10<d/10,n+1> {
    //     static_assert(d > 0, "d must be positive");
    // };
    // template <int n>
    // struct log10<1, n> : std::integral_constant<int, n> {};

    // // TODO: move this (or something like it to a common place)
    // template <class Duration = std::chrono::nanoseconds>
    // struct PrintableDuration : Duration {

    //     template <typename Rep, typename Period>
    //     PrintableDuration(const std::chrono::duration<Rep, Period>& d)
    //         : Duration(std::chrono::duration_cast<Duration>(d))
    //     {
    //     }

    //     template <typename Char, typename Traits>
    //     friend decltype(auto) operator << (std::basic_ostream<Char, Traits>& out, const PrintableDuration& d) {
    //         using Period = typename Duration::period;
    //         if constexpr (Period::num == 1 && Period::den % 10 == 0) {
    //             out << d.count() / Period::den << '.';
    //             Char oldFill = out.fill('0');
    //             auto oldWidth = out.width(log10<Period::den>::value);
    //             out << d.count() % Period::den;
    //             out.fill(oldFill);
    //             out.width(oldWidth);
    //             return out << " s";
    //         } else {
    //             return out << std::chrono::duration<double>(d).count() << " s";
    //         }
    //     }
    // };

    template <bool enable>
    struct WorkerStats;

    template <>
    struct WorkerStats<false> {
        void iteration() const {}
        void biasedSample() const {}
        void rewireTests(std::size_t) const {}
        void rewireCount() const {}
        auto& validMotion() { return TimerStat<void>::instance(); }
        auto& nearest1() { return TimerStat<void>::instance(); }
        auto& nearestK() { return TimerStat<void>::instance(); }
    };

    template <>
    struct WorkerStats<true> {
        mutable std::size_t iterations_{0};
        mutable std::size_t biasedSamples_{0};
        mutable std::size_t rewireTests_{0};
        mutable std::size_t rewireCount_{0};
        mutable TimerStat<> validMotion_;
        mutable TimerStat<> nearest1_;
        mutable TimerStat<> nearestK_;

        void iteration() const { ++iterations_; };
        void biasedSample() const { ++biasedSamples_; }
        void rewireTests(std::size_t n) const { rewireTests_ += n; }
        void rewireCount() const { ++rewireCount_; }
        TimerStat<>& validMotion() const { return validMotion_; }
        TimerStat<>& nearest1() const { return nearest1_; }
        TimerStat<>& nearestK() const { return nearestK_; }

        WorkerStats& operator += (const WorkerStats& other) {
            iterations_ += other.iterations_;
            biasedSamples_ += other.biasedSamples_;
            rewireTests_ += other.rewireTests_;
            rewireCount_ += other.rewireCount_;
            validMotion_ += other.validMotion_;
            nearest1_ += other.nearest1_;
            nearestK_ += other.nearestK_;
            return *this;
        }

        void print() const {
            MPT_LOG(INFO) << "iterations: " << iterations_;
            MPT_LOG(INFO) << "biased samples: " << biasedSamples_;
            MPT_LOG(INFO) << "rewire count: " << rewireCount_ << " of " << rewireTests_;
            MPT_LOG(INFO) << "valid motion: " << validMotion_;
            MPT_LOG(INFO) << "nearest 1: " << nearest1_;
            MPT_LOG(INFO) << "nearest K: " << nearestK_;
        }
    };

    template <typename Scenario, int maxThreads, class Rewire, bool reportStats, typename NNStrategy>
    class PRRTStar : public PlannerBase<PRRTStar<Scenario, maxThreads, Rewire, reportStats, NNStrategy>> {
        using Planner = PRRTStar;
        using Base = PlannerBase<Planner>;
        using Space = scenario_space_t<Scenario>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Link = scenario_link_t<Scenario>;
        using Traj = link_trajectory_t<Link>;
        static constexpr bool concurrent = maxThreads != 1;
        using Edge = prrt_star::Edge<State, Distance, Traj, concurrent>;
        using Node = prrt_star::Node<State, Distance, Traj, concurrent>;
        using RNG = scenario_rng_t<Scenario, Distance>;
        using Sampler = scenario_sampler_t<Scenario, RNG>;
        using Clock = std::chrono::steady_clock;

        Distance maxDistance_{std::numeric_limits<Distance>::infinity()};
        Distance goalBias_{0.01};
        Distance rewireFactor_{1.1};
        RRGRewireNeighbors<Rewire, Space> rewireNeighbors_;

        // maximum number of goals before goal bias sampling stops.
        std::size_t maxGoals_{1};

        using NNConcurrency = std::conditional_t<concurrent, nigh::Concurrent, nigh::NoThreadSafety>;
        nigh::Nigh<Node*, Space, NodeKey, NNConcurrency, NNStrategy> nn_;

        alignas(concurrent ? 64 : 0)
        Atom<Edge*, concurrent> solution_{nullptr};

        Atom<std::size_t, concurrent> goalCount_{0};

        std::mutex startNodeMutex_;
        ObjectPool<Node, false> startNodes_;
        ObjectPool<Edge, false> startEdges_;

        struct Worker;

        WorkerPool<Worker, maxThreads> workers_;

        Clock::time_point solveStartTime_;

        auto elapsedSolveTime() const {
            return Clock::now() - solveStartTime_;
        }

        void foundGoal(Edge *edge, Distance) {
            ++goalCount_;
            MPT_LOG(DEBUG) << "added goal";
            Edge *prevSolution = solution_.load(std::memory_order_acquire);
            while (prevSolution == nullptr || edge->cost() < prevSolution->cost()) {
                if (solution_.compare_exchange_weak(prevSolution, edge)) {
                    MPT_LOG(INFO) << (prevSolution
                                      ? "new solution found with cost "
                                      : "found initial solution with cost ")
                                  << edge->cost()
                                  << ", after " << elapsedSolveTime();
                    break;
                }
            }
        }

    public:
        // required constructor
        template <typename RNGSeed = RandomDeviceSeed<>>
        explicit PRRTStar(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
            : rewireNeighbors_(Sampler(scenario), scenario.space(), rewireFactor_)
            , nn_(scenario.space())
            , workers_(scenario, seed)
        {
            // calculateRewiringLowerBounds();

            MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
            MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();
        }

        void setRewireFactor(Distance factor) {
            assert(0 < factor && !std::isnan(factor));
            rewireFactor_ = factor;
            const Scenario& scenario = workers_[0].scenario();
            rewireNeighbors_ = RRGRewireNeighbors<Rewire, Space>(
                Sampler(scenario), scenario.space(), factor);
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

        // recommended, but optional method
        std::size_t size() const {
            return nn_.size();
        }

        template <typename ... Args>
        void addStart(Args&& ... args) {
            // TODO: check for validity
            std::lock_guard<std::mutex> lock(startNodeMutex_);
            bool goal = false; // TODO: check if goal
            Node *node;
            if constexpr (concurrent) {
                node = startNodes_.allocate(false, std::forward<Args>(args)...);
                workers_[0].setEdge(*this, node, startEdges_.allocate(Traj(), node));
            } else {
                node = startNodes_.allocate(false, std::forward<Args>(args)...);
            }

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

            if constexpr (reportStats)
                MPT_LOG(DEBUG) << "initial rewire: " << rewireNeighbors_.stats(nn_.size());

            MPT_LOG(DEBUG) << "range = " << maxDistance_;
            MPT_LOG(DEBUG) << "goalBias = " << goalBias_;

            solveStartTime_ = Clock::now();

            workers_.solve(*this, doneFn);

            if constexpr (reportStats) {
                MPT_LOG(DEBUG) << "final rewire: " << rewireNeighbors_.stats(nn_.size());
                if (Edge* solution = solution_.load(std::memory_order_relaxed))
                    MPT_LOG(INFO) << "final solution cost " << solution->cost();
                else
                    MPT_LOG(INFO) << "no solution found";
            }
        }

        // required method
        bool solved() const {
            return solution_.load(std::memory_order_relaxed) != nullptr;
        }

        // prototype method
        std::vector<State> solution() const {
            std::vector<State> path;
            if (const Edge *edge = solution_.load(std::memory_order_acquire)) {
                for (;;) {
                    path.push_back(edge->node()->state());
                    if ((edge = edge->parent()) == nullptr)
                        break;
                    edge = edge->node()->edge(std::memory_order_acquire);
                }
                std::reverse(path.begin(), path.end());
            }
            return path;
        }

    private:
        template <typename Fn>
        std::enable_if_t< is_trajectory_callback_v<Fn, State, Traj> >
        solutionRecur(const Edge *edge, Fn& fn) const {
            if (const Edge *p = edge->parent()) {
                solutionRecur(p, fn);
                fn(p->node()->state(), edge->link(), edge->node()->state(), true);
            }
        }

        template <typename Fn>
        std::enable_if_t< is_trajectory_reference_callback_v<Fn, State, Traj> >
        solutionRecur(const Edge *edge, Fn& fn) const {
            if (const Edge *p = edge->parent()) {
                solutionRecur(p, fn);
                fn(p->node()->state(), *edge->link(), edge->node()->state(), true);
            }
        }

        template <typename Fn>
        std::enable_if_t< is_waypoint_callback_v<Fn, State, Traj> >
        solutionRecur(const Edge *edge, Fn& fn) const {
            if (const Edge *p = edge->parent())
                solutionRecur(p, fn);
            fn(edge->node()->state());
        }

    public:
        template <typename Fn>
        void solution(Fn fn) const {
            if (const Edge *edge = solution_.load(std::memory_order_acquire))
                solutionRecur(edge, fn);
        }

        void printStats() const {
            MPT_LOG(INFO) << "nodes in graph: " << nn_.size();
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
                if (const Edge *edge = n.edge(std::memory_order_acquire))
                    if (const Edge *parent = edge->parent())
                        if (const Node *e = parent->node())
                            visitor.edge(e->state());
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

    template <typename Scenario, int maxThreads, class Rewire, bool reportStats, typename NNStrategy>
    class PRRTStar<Scenario, maxThreads, Rewire, reportStats, NNStrategy>::Worker
        : public WorkerStats<reportStats>
    {
        using Stats = WorkerStats<reportStats>;

        unsigned no_;
        Scenario scenario_;
        RNG rng_;

        ObjectPool<Node> nodes_;
        ObjectPool<Edge> edges_;

        std::vector<std::tuple<Node*, Distance>> nbh_;
        std::vector<std::tuple<Edge*, std::size_t>> edgeIndices_;

    public:
        Worker(Worker&& other)
            : scenario_(other.scenario_)
            , nodes_(std::move(other.nodes_))
            , edges_(std::move(other.edges_))
        {
        }

        template <typename RNGSeed>
        Worker(unsigned no, const Scenario& scenario, const RNGSeed& seed)
            : no_(no)
            , scenario_(scenario)
            , rng_(seed)
        {
        }

        const Scenario& scenario() const {
            return scenario_;
        }

        decltype(auto) space() const {
            return scenario_.space();
        }

        const auto& nodes() const {
            return nodes_;
        }

        template <typename DoneFn>
        void solve(Planner& planner, DoneFn done) {
            MPT_LOG(TRACE) << "worker running";

            // TODO: add "progress_interval<T>" tag, where T is a duration.
            //
            // using namespace std::literals;
            // typename Clock::duration nextProgress = 1s;

            Sampler sampler(scenario_);
            if constexpr (scenario_has_goal_sampler_v<Scenario, RNG>) {
                if (no_ == 0 && planner.goalBias_ > 0) {
                    scenario_goal_sampler_t<Scenario, RNG> goalSampler(scenario_);
                    std::uniform_real_distribution<Distance> uniform01;

                    // since we only have 1 thread performing goal
                    // biased sampling, we scale its percentage by the
                    // number of concurrent threads.
                    Distance scaledBias = planner.goalBias_ * planner.workers_.size();

                    MPT_LOG(TRACE) << "using scaled goal bias of " << scaledBias;

                    while (!done()) {
                        Stats::iteration();
                        // if ((Clock::now() - planner.solveStartTime_) > nextProgress) {
                        //     MPT_LOG(TRACE) << "size = " << planner.size() << ", biased samples = " << Stats::biasedSamples();
                        //     nextProgress += 1s;
                        // }

                        if (planner.goalCount_.load(std::memory_order_relaxed) >= 1)
                            goto unbiasedSamplingLoop;
                        if (uniform01(rng_) < planner.goalBias_) {
                            Stats::biasedSample();
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
                Stats::iteration();
                addSample(planner, sampler(rng_));
            }

            MPT_LOG(TRACE) << "worker done";
        }

        void addSample(Planner& planner, std::optional<State>&& sample) {
            if (sample)
                addSample(planner, *sample);
        }

        decltype(auto) nearest(Planner& planner, const State& q) {
            Timer timer(Stats::nearest1());
            return planner.nn_.nearest(q);
        }

        void addSample(Planner& planner, State newState) {
            // MPT_LOG(TRACE) << "q = " << randState;

            // nearest returns an optional, however it will
            // only be empty if the nn structure is empty,
            // which it will not be, because the planner's
            // solve checks first. (but we assert anyways)
            auto [nearNode, dNear] = nearest(planner, newState).value();

            // avoid adding the same state multiple times.  Unfortunately
            // this check is not sufficient, and may need to be updated.
            // numeric issues may cause distance() to return a non-zero
            // value when given the same state as both arguments.  It is
            // also possible that non-equivalent states have a zero
            // distance--but this would cause other issues with the
            // planner and thus may not be worth handling.
            if (dNear == 0)
                return;

            if (dNear > planner.maxDistance_) {
                newState = interpolate(
                    scenario_.space(),
                    nearNode->state(), newState,
                    planner.maxDistance_ / dNear);

                dNear = scenario_.space().distance(nearNode->state(), newState);
            }


            if (!scenario_.valid(newState))
                return;

            // TODO: do not need to check when scenario returns
            // std::optional<State> and the motion was not
            // interpolated.
            auto traj = validMotion(nearNode->state(), newState);
            if (!traj)
                return;

            auto [isGoal, goalDist] = scenario_goal<Scenario>::check(scenario_, newState);
            (void)goalDist; // mark unused (for now, may be used in approx solutions)

            Edge* parent = nearNode->edge(std::memory_order_relaxed);
            Distance parentCost = parent->cost() + dNear; // scenario_.space().distance(nearNode->state(), newState);

            // TODO: OMPL restricts rewiring considerations to
            // planner.maxDistance_, but only in the k-nearest
            // variant.  Their restiction seems incorrect for
            // asymptotic optimality.
            {
                Timer timer(Stats::nearestK());
                planner.rewireNeighbors_(nbh_, planner.nn_, newState);
            }

            Stats::rewireTests(nbh_.size());
            edgeIndices_.resize(nbh_.size());
            for (std::size_t i=0 ; i<nbh_.size() ; ++i)
                edgeIndices_[i] = { std::get<Node*>(nbh_[i])->edge(std::memory_order_relaxed), i };

            std::sort(
                edgeIndices_.begin(), edgeIndices_.end(),
                [&] (const auto& a, const auto& b) {
                    return std::get<Edge*>(a)->cost() + std::get<Distance>(nbh_[std::get<std::size_t>(a)])
                         < std::get<Edge*>(b)->cost() + std::get<Distance>(nbh_[std::get<std::size_t>(b)]);
                });

            // Find the nearest parent to connect, since edges are
            // sorted from nearest the farthest, we can stop as soon
            // as we've found a valid connection, or we've found the
            // nearNode that we've already checked.
            for (auto [nbrEdge, nbrIndex] : edgeIndices_) {
                Distance newCost = nbrEdge->cost() + std::get<Distance>(nbh_[nbrIndex]);

                if (nbrEdge == parent && newCost != parentCost) {
                    MPT_LOG(FATAL) << "bad distance: " << newCost << " != " << parentCost;
                    abort();
                }

                if (newCost > parentCost)
                    break;

                std::get<Node*>(nbh_[nbrIndex]) = nullptr; // mark as checked

                if (nbrEdge->node() == nearNode) {
                    parent = nbrEdge;
                    parentCost = newCost;
                    break;
                }

                if (auto nbrTraj = validMotion(nbrEdge->node()->state(), newState)) {
                    traj = nbrTraj;
                    parent = nbrEdge;
                    parentCost = newCost;
                    break;                    
                }
            }

            Node* newNode;
            Edge* newEdge;

            if constexpr (concurrent) {
                newNode = nodes_.allocate(isGoal, newState);
                newEdge = edges_.allocate(linkTrajectory(traj), newNode, parent, parentCost);
                setEdge(planner, newNode, newEdge);
            } else {
                newNode = nodes_.allocate(linkTrajectory(traj), parent, parentCost, isGoal, newState);
                newEdge = newNode->edge();
            }

            planner.nn_.insert(newNode);

            if (isGoal)
                planner.foundGoal(newEdge, goalDist);

            // rewire from nearest to farthest (TODO: for PRRT, this
            // should be done in reverse)
            for (auto [nbrNode, nbrDist] : nbh_) {
                // nbrNode will be set to null in the first pass if it
                // was already checked.
                if (nbrNode == nullptr)
                    continue;

                assert(nbrNode != parent->node());

                Edge *nbrEdge = nbrNode->edge(std::memory_order_acquire);
                Distance newCost = parentCost + nbrDist;
                if (newCost >= nbrEdge->cost())
                    continue;
                
                if (auto traj = validMotion(newNode->state(), nbrNode->state())) {
                    if constexpr (concurrent) {
                        setEdge(planner, nbrNode, edges_.allocate(
                                    linkTrajectory(traj), nbrNode, newEdge, newCost));
                    } else {
                        // we special case the update for
                        // non-concurrent planning (i.e. standard
                        // RRT*) In this case, we can reuse the
                        // existing edges without worry of a
                        // concurrent update.
                        Distance delta = nbrEdge->cost() - newCost;
                        nbrEdge->setLink(linkTrajectory(traj));
                        nbrEdge->setParent(newEdge);
                        nbrEdge->setCost(newCost);
                        nonConcurrentPushUpdate(planner, nbrEdge, delta);
                    }
                }
            }
        }

        decltype(auto) validMotion(const State& a, const State& b) {
            Timer timer(Stats::validMotion());
            return scenario_.link(a, b);
        }

        void nonConcurrentPushUpdate(Planner& planner, Edge* edge, Distance delta) {
            assert(!concurrent && delta > 0);
            Stats::rewireCount();
            if (edge->node()->goal()) {
                Edge *prevSolution = planner.solution_;
                if (edge == prevSolution) {
                    MPT_LOG(INFO) << "solution improved, new cost "
                        << edge->cost()
                        << ", after " << planner.elapsedSolveTime();
                } else if (edge->cost() < prevSolution->cost()) {
                    planner.solution_.store(edge);
                    MPT_LOG(INFO) << "solution changed, new cost "
                        << edge->cost()
                        << ", after " << planner.elapsedSolveTime();
                }
            }

            for (Edge *child = edge->firstChild(std::memory_order_relaxed) ;
                 child != nullptr ;
                 child = child->nextSibling(std::memory_order_acquire))
            {
                child->setCost(child->cost() - delta);
                nonConcurrentPushUpdate(planner, child, delta);
            }
        }

        void setEdge(Planner& planner, Node* node, Edge* newEdge) {
            Edge *oldEdge = node->edge(std::memory_order_relaxed);
            for (;;) {
                if (oldEdge && oldEdge->cost() <= newEdge->cost()) {
                    // the existing edge is shorter, move in reverse
                    std::swap(oldEdge, newEdge);
                    break;
                }
                if (node->casEdge(
                        oldEdge, newEdge,
                        std::memory_order_release,
                        std::memory_order_relaxed))
                    break;
                // MPT_LOG(DEBUG, "CAS failed (setEdge)", no_);
            }

            if (node->goal()) {
                // note: prevSolution can be null under concurrency,
                // the goal node is inserted into the motion graph
                // before it updates the solution.
                Edge *prevSolution = planner.solution_.load(std::memory_order_acquire);
                while (prevSolution == nullptr || newEdge->cost() < prevSolution->cost()) {
                    if (planner.solution_.compare_exchange_weak(
                            prevSolution, newEdge,
                            std::memory_order_release,
                            std::memory_order_relaxed))
                    {
                        MPT_LOG(INFO) << (prevSolution == nullptr
                                          ? "found initial solution with cost "
                                          : (newEdge->node() == prevSolution->node()
                                             ? "solution improved, new cost "
                                             : "solution changed, new cost "))
                                      << newEdge->cost()
                                      << ", after " << planner.elapsedSolveTime();
                        break;
                    } else {
                        // MPT_LOG(DEBUG, "[%u]: CAS failed (update solution)", no_);
                    }
                }
            }

            // at this point, oldEdge is "owned" by this thread,
            // whether or not the CAS was successful.
            if (oldEdge == nullptr)
                return;

            do {
                Distance costDelta = oldEdge->cost() - newEdge->cost();
                assert(costDelta >= 0);

                // remove the children from the oldEdge.  Another thread
                // may still have a reference to it.
                Edge *firstChild = oldEdge->firstChild(std::memory_order_relaxed);
                while (!oldEdge->casFirstChild(
                           firstChild, nullptr,
                           std::memory_order_release,
                           std::memory_order_relaxed)) {
                    // MPT_LOG(DEBUG, "[%u]: CAS failed (firstChild)", no_);
                }

                for (Edge *oldChild = firstChild ; oldChild ; oldChild = oldChild->nextSibling(std::memory_order_acquire)) {
                    Node *childNode = oldChild->node();
                    Edge *shorterEdge = edges_.allocate(
                        *oldChild, childNode, newEdge, oldChild->cost() - costDelta);
                    
                    setEdge(planner, childNode, shorterEdge);
                }

                // we've moved all the children from oldEdge to
                // newEdge.  Check now that newEdge is still active.
                // If it is not, then we must move over any remaining
                // edges to the replacement.

                oldEdge = newEdge;
                newEdge = node->edge(std::memory_order_acquire);
            } while (oldEdge != newEdge);
        }
    };
}

#endif
