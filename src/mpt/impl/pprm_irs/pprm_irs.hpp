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
#ifndef MPT_IMPL_PPRM_IRS_PLANNER_HPP
#define MPT_IMPL_PPRM_IRS_PLANNER_HPP

#include "component.hpp"
#include "node.hpp"
#include "edge.hpp"
#include "shortest_path_check.hpp"
#include "../planner_base.hpp"
#include "../scenario_rng.hpp"
#include "../scenario_sampler.hpp"
#include "../scenario_goal.hpp"
#include "../worker_pool.hpp"
#include "../goal_has_sampler.hpp"
#include "../../random_device_seed.hpp"
#include "../object_pool.hpp"
#include <forward_list>
#include <vector>

namespace unc::robotics::mpt::impl::pprm_irs {
    template <typename Scenario, int maxThreads, bool keepDense, bool reportStats, typename NNStrategy>
    class PPRMIRS : public PlannerBase<PPRMIRS<Scenario, maxThreads, keepDense, reportStats, NNStrategy>> {
        using Planner = PPRMIRS;
        using Base = PlannerBase<Planner>;
        using Space = scenario_space_t<Scenario>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Node = pprm_irs::Node<State, Distance, keepDense>;
        using Edge = pprm_irs::Edge<State, Distance, keepDense>;
        using EdgePair = pprm_irs::EdgePair<State, Distance, keepDense>;
        using RNG = scenario_rng_t<Scenario, Distance>;
        using Sampler = scenario_sampler_t<Scenario, RNG>;

        using NNConcurrency = std::conditional_t<maxThreads == 1, nigh::NoThreadSafety, nigh::Concurrent>;
        nigh::Nigh<Node*, Space, NodeKey, NNConcurrency, NNStrategy> nn_;

        struct Worker;

        WorkerPool<Worker, maxThreads> workers_;
        std::atomic_bool solved_{false};

        Distance stretchWeight_{5};
        Distance kRRG_;

        std::mutex mutex_;
        std::forward_list<Node*> startNodes_;
        std::set<const Node*> goalNodes_;

        void foundGoal(Node *node) {
            // TODO: if there are a lot of goals, then this could
            // become a concurrency bottleneck.  We can replace it
            // with a lock-free linked list.
            MPT_LOG(TRACE) << "found goal";
            std::lock_guard<std::mutex> lock(mutex_);
            goalNodes_.insert(node);
        }

        void solutionFound() {
            bool wasSolved = solved_.load(std::memory_order_relaxed);
            if (!wasSolved && solved_.compare_exchange_strong(wasSolved, true, std::memory_order_relaxed))
                MPT_LOG(INFO) << "solution found";
        }

    public:
        template <typename RNGSeed = RandomDeviceSeed<>>
        PPRMIRS(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
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

        // Sets the stretch weight.  It may be okay to change this
        // while another thread (group) is solving, but the behavior
        // is not specified.
        void setStretchWeight(Distance w) {
            stretchWeight_ = w;
        }

        template <typename ... Args>
        void addStart(Args&& ... args) {
            Node *n = workers_[0].addSample(*this, State(std::forward<Args>(args)...), Component::kStart);

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
            using Goal = scenario_goal_t<Scenario>;
            if constexpr (goal_has_sampler_v<Goal>)
                if (goalNodes_.empty())
                    workers_[0].sampleGoals(*this);

            if (goalNodes_.empty() || startNodes_.empty())
                throw std::runtime_error("PPRM requires both start and goal configurations");

            workers_.solve(*this, doneFn);
        }

        bool solved() const {
            return solved_.load(std::memory_order_relaxed);
        }

        std::vector<State> solution() const {
            using QItem = std::tuple<Distance, const Node*>;
            auto compare = [] (const QItem& a, const QItem& b) { return std::get<0>(a) > std::get<0>(b); };
            std::unordered_map<const Node*, QItem> nodeInfo;
            std::priority_queue<QItem, std::vector<QItem>, decltype(compare)> q(compare);

            for (const Node* n : startNodes_) {
                nodeInfo[n] = std::tuple<Distance, const Node*>(0, nullptr);
                q.emplace(Distance(0), n);
            }

            std::vector<State> path;
            while (!q.empty()) {
                auto [ dMin, min ] = q.top();
                q.pop();

                // already popped
                if (std::get<Distance>(nodeInfo[min]) < dMin)
                    continue;

                assert(std::get<Distance>(nodeInfo[min]) == dMin);

                if (goalNodes_.find(min) != goalNodes_.end()) {
                    MPT_LOG(DEBUG) << "goal expaned";
                    for (const Node *n = min ; n ; n = std::get<const Node*>(nodeInfo[n]))
                        path.push_back(n->state());
                    std::reverse(path.begin(), path.end());
                    break;
                }
                
                for (const Edge *e = min->sparseHead(std::memory_order_acquire) ; e ; e = e->next(std::memory_order_acquire)) {
                    Distance d = dMin + workers_[0].space().distance(min->state(), e->to()->state());
                    auto dBest = nodeInfo.find(e->to());
                    if (dBest == nodeInfo.end()) {
                        nodeInfo[e->to()] = {d, min};
                        q.emplace(d, e->to());
                    } else if (d < std::get<Distance>(dBest->second)) {
                        dBest->second = {d, min}; // std::make_tuple(d, min);
                        q.emplace(d, e->to());
                    }
                }
            }

            return path;
        }

        void printStats() const {
            MPT_LOG(INFO) << "nodes in graph: " << nn_.size();
        }

        template <typename Visitor>
        void visitGraph(Visitor&& visitor) const {
            for (const Worker& worker : workers_)
                worker.visitGraph(std::forward<Visitor>(visitor));
        }
    };

    template <typename Scenario, int maxThreads, bool keepDense, bool reportStats, typename NNStrategy>
    class PPRMIRS<Scenario, maxThreads, keepDense, reportStats, NNStrategy>::Worker {
        unsigned no_;
        Scenario scenario_;
        RNG rng_;

        ObjectPool<Node> nodePool_;
        ObjectPool<EdgePair> edgePool_;
        ObjectPool<Component> componentPool_;

        std::vector<std::tuple<Distance, Node*>> nbh_;

        ShortestPathCheck<Space, keepDense> shortestPathCheck_;

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
            using Goal = scenario_goal_t<Scenario>;
            GoalSampler<Goal> goalSampler(scenario_.goal());
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
            } else if ((isGoal = scenario_.goal()(scenario_.space(), q).first) == true) {
                flags = static_cast<Component::Flags>(flags | Component::kGoal);
            }

            Component *component = componentPool_.allocate(1, flags);
            Node *n = nodePool_.allocate(component, q);

            if (isGoal)
                planner.foundGoal(n);

            // Reset the shortest path check since all neighbors start
            // from the same source point in our search and we can
            // reuse information from one check to the next.
            shortestPathCheck_.reset(n);
            
            for (auto [d, nbr] : nbh_) {
                if (!validMotion(q, nbr->state()))
                    continue;

                addEdge(planner, n, nbr, d);
            }

            planner.nn_.insert(n);
            return n;
        }

        void addEdge(Planner& planner, Node* from, Node *to, Distance d) {
            Distance stretchDist = planner.stretchWeight_ * d;
            if (shortestPathCheck_(from, to, stretchDist, scenario_.space())) {
                // sparse
                EdgePair *pair = edgePool_.allocate(from, to, d);
                from->addSparseEdge(pair->get(1));
                to->addSparseEdge(pair->get(0));
            } else if constexpr (keepDense) {
                EdgePair *pair = edgePool_.allocate(from, to, d);
                from->addDenseEdge(pair->get(1));
                to->addDenseEdge(pair->get(0));
            } else {
                return;
            }

            Component *cm = merge(from->component(), to->component());
            if (cm->isSolution())
                planner.solutionFound();
        }

        Component *merge(Component *a, Component *b) {
            Component *t;
            do {
                while ((t = a->next()) != nullptr) a = t;
                while ((t = b->next()) != nullptr) b = t;
                if (a == b) return a;
                if (a->size() > b->size())
                    std::swap(a, b);
                assert(t == nullptr);
            } while (!a->casNext(t, b, std::memory_order_relaxed));

            Component *m = componentPool_.allocate(a, b);
            while (!b->casNext(t, m, std::memory_order_relaxed)) {
                while ((t = b->next()) != nullptr) b = t;
                m->update(a, b);
            }

            return m;
        }

        bool validMotion(const State& a, const State& b) {
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
                for (const Edge *e = n.sparseHead(std::memory_order_acquire) ; e ;
                     e = e->next(std::memory_order_relaxed))
                    visitor.edge(e->to()->state());
            }
        }
    };
}

#endif
