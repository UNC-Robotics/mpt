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
#ifndef MPT_IMPL_PPRM_IRS_SHORTEST_PATH_CHECK_HPP
#define MPT_IMPL_PPRM_IRS_SHORTEST_PATH_CHECK_HPP

#include "node.hpp"
#include "edge.hpp"
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace unc::robotics::mpt::impl::pprm_irs {
    template <typename Space, bool keepDense>
    class ShortestPathCheck {
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Node = pprm_irs::Node<State, Distance, keepDense>;
        using Edge = pprm_irs::Edge<State, Distance, keepDense>;

        using Iteration = unsigned;
        
        using PathCost = std::pair<Distance, Iteration>;
        using QueueItem = std::pair<Distance, const Node*>;

        struct MinHeapCompare {
            bool operator() (const QueueItem& a, const QueueItem& b) {
                return a.first >= b.first;
            }
        };

        // This is used to see if an item's mapped value is out of
        // date by checking against PathCost's iter_.  We can actually
        // leave this uninitialized.
        Iteration iter_{0};

        const Node *from_;
        std::unordered_map<const Node*, PathCost> pathCosts_;
        std::vector<QueueItem> pathQueue_;

    public:
        void clear() {
            iter_ = 0;
            from_ = nullptr;
            pathCosts_.clear();
        }

        void reset(const Node *u) {
            from_ = u;

            // Unfortunately clearing the pathCosts_ leads to
            // significant slowdown (in 2D holonomic test case, having
            // pathCosts_.clear() will result in 1100 vertices,
            // whereas we get 4400 without).  So instead, we use an
            // iter count to track which entries in the map are stale.

            ++iter_;

            pathQueue_.clear();
            updateQueue(u, 0);
        }

        void updateQueue(const Node *n, Distance cost) {
            pathCosts_[n] = { cost, iter_ };
            // auto entry = pathCosts_.find(n);
            // if (entry == pathCosts_.end()) {
            //     pathCosts_.emplace(
            //         std::piecewise_construct,
            //         std::forward_as_tuple(n),
            //         std::forward_as_tuple(cost, iter_));
            // } else {
            //     entry->second = { cost, iter_ };
            // }
            pathQueue_.emplace_back(cost, n);
        }

        // Compute the shortest path from `u` to `v`, checking to see
        // if the distance is shorter or longer than the target
        // distance.  If the shortest path is shorter than the
        // distance, then we have a sufficiently short path between
        // the two points already and we determine that we do not need
        // a sparse edge between the two.
        bool operator() (const Node *u, const Node *v, Distance distTarget, const Space& space) {
            // The caller should have called 'reset(u);' prior to
            // making this call.  Here we just perform a sanity check
            // on the argument to ensure that the caller is doing what
            // they're supposed to do.
            assert(from_ == u);

            // Quick check to see if we've already expanded the path
            // being queried.
            auto vPathCost = pathCosts_.find(v);
            if (vPathCost != pathCosts_.end() && vPathCost->second.second == iter_) {
                // check if we already have a path to v with a
                // distance less than the target, thus we know we do
                // not need a sparse edge.
                if (vPathCost->second.first < distTarget)
                    return false;
            }

            assert(!pathQueue_.empty());

            while (!pathQueue_.empty()) {
                auto [ topPriority, top ] = pathQueue_.front();

                PathCost& topPathCost = pathCosts_.at(top);

                // nothing in the queue should be from a previous
                // iteration since we clear the queue on reset.
                assert(topPathCost.second == iter_);
                
                Distance pathCost = topPathCost.first;

                // we've have not encountred v, and we're already more
                // than the distance target away.  This implies that
                // there is either no path to v, or the distance to v
                // is greater than the target.  Thus we need a sparse
                // edge.
                if (pathCost >= distTarget)
                    break;

                std::pop_heap(pathQueue_.begin(), pathQueue_.end(), MinHeapCompare{});
                pathQueue_.pop_back();

                // check for a stale entry
                if (topPathCost.first != topPriority) {
                    // only reason it should be stale is if we already
                    // popped it off because it had a shorter path
                    assert(topPathCost.first < topPriority);
                    continue;
                }

                // check if we've encountered `v`, then we know we
                // have a pathCost less than the distance target, and
                // we're done.
                bool found = (top == v);
                for (const Edge *edge = top->sparseHead(std::memory_order_acquire) ;
                     edge != nullptr ;
                     edge = edge->next(std::memory_order_acquire))
                {
                    const Node *nbr = edge->to();
                    Distance d = pathCost + edge->distance();

                    if (nbr == v && d < distTarget)
                        found = true;

                    auto nbrPathCost = pathCosts_.find(nbr);
                    if (nbrPathCost == pathCosts_.end()) {
                        // first time we've encountered nbr in any
                        // iteration
                        nbrPathCost = pathCosts_.emplace(
                            std::piecewise_construct,
                            std::forward_as_tuple(nbr),
                            std::forward_as_tuple(d, iter_))
                            .first;
                    } else if (nbrPathCost->second.second != iter_) {
                        // first time we've encountered nbr in this
                        // search
                        nbrPathCost->second.first = d;
                        nbrPathCost->second.second = iter_;
                    } else if (d < nbrPathCost->second.first) {
                        // found a shorter path
                        nbrPathCost->second.first = d;
                    } else {
                        // existing path in queue is better then
                        // current option, do not update the queue.
                        continue;
                    }
                    assert(nbrPathCost->second.first > 0);
                    pathQueue_.emplace_back(nbrPathCost->second.first, nbr);
                    std::push_heap(pathQueue_.begin(), pathQueue_.end(), MinHeapCompare{});
                }

                // check and terminate if we found a path to v with a
                // distance less than the target, and we do not need a
                // sparse edge.
                if (found)
                    return false;
            }

            // if the queue is empty then there is no path to v, and
            // we need a sparse edge.  We're also here if the pathCost
            // >= distTarget.

            // Since we're adding a sparse edge to the graph, we
            // update the queue for the next call
            updateQueue(v, space.distance(from_->state(), v->state()));
            std::push_heap(pathQueue_.begin(), pathQueue_.end(), MinHeapCompare{});

            return true;
        }
    };    
}

#endif
