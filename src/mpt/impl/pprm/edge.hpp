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
#ifndef MPT_IMPL_PPRM_EDGE_HPP
#define MPT_IMPL_PPRM_EDGE_HPP

#include "../link.hpp"
#include <atomic>

namespace unc::robotics::mpt::impl::pprm {
    template <typename State, typename Distance, typename Traj>
    class Node;

    template <typename State, typename Distance, typename Traj>
    class EdgePair;

    template <typename State, typename Distance, typename Traj>
    class Edge {
        using Node = pprm::Node<State, Distance, Traj>;
        using EdgePair = pprm::EdgePair<State, Distance, Traj>;
        
        Node *to_;
        EdgePair *pair_;
        std::atomic<Edge*> next_{nullptr};

    public:
        Edge(Node *to, EdgePair *pair)
            : to_(to)
            , pair_(pair)
        {
        }

        Distance distance() const {
            return pair_->distance();
        }

        bool forward() const {
            return pair_->get(0) == this;
        }

        decltype(auto) link() const {
            return pair_->link();
        }

        const Node* to() const {
            return to_;
        }

        const Node* from() const {
            return pair_->other(this)->to_;
        }
        
        void setNext(Edge *next, std::memory_order order) {
            assert(next == nullptr || next->from() == from());
            next_.store(next, order);
        }

        const Edge* next(std::memory_order order) const {
            return next_.load(order);
        }
        
    };
    
    template <typename State, typename Distance, typename Traj>
    class EdgePair : public Link<Traj> {
        using Node = pprm::Node<State, Distance, Traj>;
        using Edge = pprm::Edge<State, Distance, Traj>;

        Distance distance_;
        std::array<Edge, 2> pair_;

    public:
        EdgePair(Node *from, Node* to, Distance dist, Traj&& traj)
            : Link<Traj>(std::move(traj))
            , distance_(dist)
            , pair_{{{to, this}, {from, this}}}
        {
        }

        Edge* get(int i) {
            return &pair_[i];
        }

        const Edge* get(int i) const {
            return &pair_[i];
        }

        Distance distance() const {
            return distance_;
        }

        const Edge* other(const Edge* half) const {
            assert(half == &pair_[0] || half == &pair_[1]);
            return &pair_[half == &pair_[0]];
        }
    };

    template <typename Edge>
    static void linkEdge(std::atomic<Edge*>& head, Edge *edge) {
        Edge* oldHead = head.load(std::memory_order_relaxed);
        do {
            edge->setNext(oldHead, std::memory_order_relaxed);
        } while (head.compare_exchange_weak(
                     oldHead, edge,
                     std::memory_order_release,
                     std::memory_order_relaxed));
    }
}

#endif

