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
#ifndef MPT_IMPL_PPRM_NODE_HPP
#define MPT_IMPL_PPRM_NODE_HPP

#include <atomic>

namespace unc::robotics::mpt::impl::pprm {
    template <typename State, typename Distance, typename Traj>
    class Edge;

    template <typename State, typename Distance, typename Traj>
    class Node {
        State state_;
        std::atomic<Component*> component_;
        std::atomic<Edge<State, Distance, Traj>*> edges_;
    public:
        template <typename ... Args>
        Node(Component *component, Args&& ... args)
            : state_(std::forward<Args>(args)...)
            , component_(component)
            , edges_(nullptr)
        {
        }

        const State& state() const {
            return state_;
        }

        const Edge<State, Distance, Traj>* edges() const {
            return edges_.load(std::memory_order_acquire);
        }

        Component* component() {
            Component *c = component_.load(std::memory_order_relaxed);
            if (Component *n = c->next()) {
                component_.compare_exchange_strong(c, n, std::memory_order_relaxed);
                return n;
            }
            return c;
        }

        Component* addEdge(Edge<State, Distance, Traj> *edge) {
            Edge<State, Distance, Traj> *head = edges_.load(std::memory_order_relaxed);
            do {
                edge->setNext(head, std::memory_order_relaxed);
            } while (edges_.compare_exchange_weak(
                         head, edge,
                         std::memory_order_release,
                         std::memory_order_relaxed));
            return component();
        }
    };

    struct NodeKey {
        template <typename State, typename Distance, typename Traj>
        const State& operator() (const Node<State, Distance, Traj>* n) const {
            return n->state();
        }
    };
}

#endif

