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
#ifndef MPT_IMPL_PPRM_PLANNER_HPP
#define MPT_IMPL_PPRM_PLANNER_HPP

#include "component.hpp"
#include "node.hpp"
#include "edge.hpp"
#include "../djikstras.hpp"
#include "../goal_has_sampler.hpp"
#include "../link_trajectory.hpp"
#include "../object_pool.hpp"
#include "../planner_base.hpp"
#include "../scenario_goal.hpp"
#include "../scenario_goal_sampler.hpp"
#include "../scenario_link.hpp"
#include "../scenario_rng.hpp"
#include "../scenario_sampler.hpp"
#include "../scenario_space.hpp"
#include "../worker_pool.hpp"
#include "../../goal_sampler.hpp"
#include "../../random_device_seed.hpp"
#include <mutex>
#include <atomic>
#include <forward_list>
#include <set>
#include <unordered_map>

namespace unc::robotics::mpt::impl::pprm {

    template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
    class PPRM : public PlannerBase<PPRM<Scenario, maxThreads, reportStats, NNStrategy>> {
        using Planner = PPRM;
        using Base = PlannerBase<PPRM>;
        using Space = scenario_space_t<Scenario>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Link = scenario_link_t<Scenario>;
        using Traj = link_trajectory_t<Link>;
        using Node = pprm::Node<State, Distance, Traj>;
        using Edge = pprm::Edge<State, Distance, Traj>;
        using EdgePair = pprm::EdgePair<State, Distance, Traj>;
        using RNG = scenario_rng_t<Scenario, Distance>;
        using Sampler = scenario_sampler_t<Scenario, RNG>;

        using NNConcurrency = std::conditional_t<maxThreads == 1, nigh::NoThreadSafety, nigh::Concurrent>;
        nigh::Nigh<Node*, Space, NodeKey, NNConcurrency, NNStrategy> nn_;

        struct Worker;

        WorkerPool<Worker, maxThreads> workers_;
        std::atomic_bool solved_{false};

        Distance kRRG_;

        std::mutex mutex_;
        std::forward_list<Node*> startNodes_;
        std::set<const Node*> goalNodes_;

        // TODO: add to stats:
        // std::atomic_uint componentCount_{0};

        void foundGoal(Node *node) {
            // TODO: if there are a lot of goals, then this could
            // become a concurrency bottleneck.  We can replace it
            // with a lock-free linked list.
            MPT_LOG(TRACE) << "found goal";
            assert(node->component()->isGoal());
            std::lock_guard<std::mutex> lock(mutex_);
            goalNodes_.insert(node);
        }

        void solutionFound() {
            bool wasSolved = solved_.load(std::memory_order_relaxed);
            if (!wasSolved && solved_.compare_exchange_strong(wasSolved, true, std::memory_order_relaxed))
                MPT_LOG(INFO) << "solution found";
        }

        // Functor used by shortest path algorithm
        struct Goal {
            const std::set<const Node*>& goalNodes_;
            
            bool operator() (const Node* n) const {
                return goalNodes_.count(n) > 0;
            }
            
            bool operator() (const Edge* e) const {
                return goalNodes_.count(e->to()) > 0;
            }
        };

        // Functor used by shortest path algorithm
        struct Edges {
            template <typename Callback>
            void operator() (const Node *from, Callback callback) const {
                for (const Edge *e = from->edges() ; e ; e = e->next(std::memory_order_acquire))
                    callback(e->distance(), e->to());
            }
            
            template <typename Callback>
            void operator() (const Edge *from, Callback callback) const {
                for (const Edge *e = from->to()->edges() ; e ; e = e->next(std::memory_order_acquire))
                    callback(e->distance(), e);
            }
        };

    public:
        template <typename RNGSeed = RandomDeviceSeed<>>
        PPRM(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
            : nn_(scenario.space())
            , workers_(scenario, seed)
            , kRRG_(E<Distance> + E<Distance> / scenario.space().dimensions())
        {
            MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
            MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();
        }

        std::size_t size() const {
            return nn_.size();
        }

        template <typename ... Args>
        void addStart(Args&& ... args) {
            Node *n = workers_[0].addSample(*this, State(std::forward<Args>(args)...), Component::kStart);

            assert(n->component()->isStart());

            std::lock_guard<std::mutex> lock(mutex_);
            startNodes_.push_front(n);
        }

        template <typename ... Args>
        void addGoal(Args&& ... args) {
            workers_[0].addSample(*this, State(std::forward<Args>(args)...), Component::kGoal);
        }

        // required method
        template <typename DoneFn>
        std::enable_if_t<std::is_same_v<bool, std::result_of_t<DoneFn()>>>
        solve(DoneFn doneFn) {
            if constexpr (scenario_has_goal_sampler_v<Scenario, RNG>)
                if (goalNodes_.empty())
                    workers_[0].sampleGoals(*this);

            if (goalNodes_.empty() || startNodes_.empty())
                throw std::runtime_error("PPRM requires both start and goal configurations");

            workers_.solve(*this, doneFn);
        }

        bool solved() const {
            return solved_.load(std::memory_order_relaxed);
        }

        template <typename Fn>
        std::enable_if_t<
            is_trajectory_callback_v<Fn, State, Traj> ||
            is_trajectory_reference_callback_v<Fn, State, Traj> >
        solution(Fn fn) const {
            Djikstras<const Edge*, Distance, Goal, Edges> shortestPath{Goal{goalNodes_}};
            
            for (const Node* n : startNodes_) {
                if (goalNodes_.count(n) > 0) {
                    MPT_LOG(WARN, "start is goal, cannot iterate through trajectories");
                    return;
                }
                for (const Edge *e = n->edges() ; e ; e = e->next(std::memory_order_acquire))
                    shortestPath.addStart(e, e->distance());
            }

            shortestPath([&] (std::size_t n, auto first, auto last) {
                if (n < 2)
                    return;
                for (auto it = first ; it != last ; ++it) {
                    const Edge *e = *it;
                    if constexpr (is_trajectory_reference_callback_v<Fn, State, Traj>) {
                        fn(e->from()->state(), *e->link(), e->to()->state(), e->forward());
                    } else {
                        fn(e->from()->state(), e->link(), e->to()->state(), e->forward());
                    }
                }
            });
        }
        
        template <typename Fn>
        std::enable_if_t< is_waypoint_callback_v<Fn, State, Traj> >
        solution(Fn fn) const {
            djikstras<const Node*, Distance>(
                startNodes_.begin(), startNodes_.end(), Goal{goalNodes_}, Edges{},
                [&] (std::size_t n, auto first, auto last) {
                    for (auto it = first ; it != last ; ++it)
                        fn((*it)->state());
                });
        }

        std::vector<State> solution() const {
            return djikstras<const Node*, Distance>(
                startNodes_.begin(), startNodes_.end(), Goal{goalNodes_}, Edges{},
                [&] (std::size_t n, auto first, auto last) {
                    std::vector<State> result;
                    result.reserve(n);
                    for (auto it = first ; it != last ; ++it)
                        result.push_back((*it)->state());
                    return result;
                });
        }        
        
        void printStats() const {
            MPT_LOG(INFO) << "nodes in graph: " << nn_.size();
            // MPT_LOG(INFO) << "component count: " << componentCount_.load();
        }

        template <typename Visitor>
        void visitGraph(Visitor&& visitor) const {
            for (const Worker& worker : workers_)
                worker.visitGraph(std::forward<Visitor>(visitor));
        }
    };

    template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
    class PPRM<Scenario, maxThreads, reportStats, NNStrategy>::Worker {
        unsigned no_;
        Scenario scenario_;
        RNG rng_;

        ObjectPool<Node> nodePool_;
        ObjectPool<EdgePair> edgePool_;
        ObjectPool<Component> componentPool_;

        std::vector<std::tuple<Distance, Node*>> nbh_;

    public:
        Worker(Worker&& other)
            : no_(other.no_)
            , scenario_(std::move(other.scenario_))
            , rng_(std::move(other.rng_))
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

        const Space& space() const {
            return scenario_.space();
        }

        void sampleGoals(Planner& planner) {
            // TODO: more than one sample when appropriate
            scenario_goal_sampler_t<Scenario, RNG> goalSampler(scenario_);
            addSample(planner, goalSampler(rng_), Component::kGoal);
        }

        void addSample(Planner& planner, std::optional<State>&& sample, Component::Flags flags) {
            if (sample)
                addSample(planner, *sample, flags);
        }

        Node* addSample(Planner& planner, const State& q, Component::Flags flags) {
            if (!scenario_.valid(q))
                return nullptr;

            Distance logSizePlus1 = std::log(planner.nn_.size() + 1);
            int k = std::ceil(planner.kRRG_ * logSizePlus1);
            planner.nn_.nearest(nbh_, q, k);

            Distance minDist = std::numeric_limits<Distance>::epsilon();
            if (!nbh_.empty() && std::get<Distance>(nbh_[0]) < minDist)
                return nullptr;

            bool isGoal;

            if ((flags & Component::kGoal) != 0) {
                isGoal = true;
            } else if ((isGoal = scenario_goal<Scenario>::check(scenario_, q).first) == true) {
                flags = static_cast<Component::Flags>(flags | Component::kGoal);
            }

            Component *component = componentPool_.allocate(1, flags);
            // ++planner.componentCount_;
            Node *n = nodePool_.allocate(component, q);

            if (isGoal)
                planner.foundGoal(n);

            for (auto [d, nbr] : nbh_) {
                if (auto traj = validMotion(q, nbr->state())) {
                    EdgePair *pair = edgePool_.allocate(n, nbr, d, linkTrajectory(traj));
                    Component *c0 = n->addEdge(pair->get(0));
                    Component *c1 = nbr->addEdge(pair->get(1));
                    Component *cm = merge(planner, c0, c1);

                    if (cm->isSolution())
                        planner.solutionFound();
                }
            }

            planner.nn_.insert(n);
            return n;
        }

        Component *merge(Planner& planner, Component *a, Component *b) {
            Component *t;
            do {
                while ((t = a->next()) != nullptr) a = t;
                while ((t = b->next()) != nullptr) b = t;
                if (a == b) return a; // same component already
                if (a->size() > b->size())
                    std::swap(a, b);
                assert(t == nullptr);
            } while (!a->casNext(t, b, std::memory_order_relaxed));
            
            // component merged
            // --planner.componentCount_;

            Component *m = componentPool_.allocate(a, b);
            while (!b->casNext(t, m, std::memory_order_relaxed)) {
                while ((t = b->next()) != nullptr) b = t;
                m->update(a, b);
            }

            return m;
        }

        decltype(auto) validMotion(const State& a, const State& b) {
            return scenario_.link(a, b);
        }

        template <typename DoneFn>
        void solve(Planner& planner, DoneFn done) {
            MPT_LOG(TRACE) << "worker running";

            Sampler sampler(scenario_);
            while (!done()) {
                addSample(planner, sampler(rng_), Component::kNone);
            }

            MPT_LOG(TRACE) << "worker done";
        }

        template <typename Visitor>
        void visitGraph(Visitor&& visitor) const {
            for (const Node& n : nodePool_) {
                visitor.vertex(n.state());
                for (const Edge *e = n.edges() ; e ; e = e->next(std::memory_order_acquire))
                    visitor.edge(e->to()->state());
            }
        }
    };
}

#endif
