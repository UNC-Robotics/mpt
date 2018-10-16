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
#ifndef MPT_IMPL_PRRT_STAR_NODE_HPP
#define MPT_IMPL_PRRT_STAR_NODE_HPP

#include "../atom.hpp"

namespace unc::robotics::mpt::impl::prrt_star {
    template <typename State, typename Distance, typename Traj, bool concurrent>
    class Edge;

    template <typename State, typename Distance, typename Traj, bool concurrent>
    class Node {
        using Edge = prrt_star::Edge<State, Distance, Traj, concurrent>;

        State state_;
        Atom<Edge*, concurrent> edge_{nullptr};
        bool goal_;

    public:
        template <typename ... Args>
        Node(bool goal, Args&& ... args)
            : state_(std::forward<Args>(args)...)
            , goal_(goal)
        {
        }

        bool goal() const {
            return goal_;
        }

        Edge* edge(std::memory_order order) {
            return edge_.load(order);
        }

        const Edge* edge(std::memory_order order) const {
            return edge_.load(order);
        }

        void setEdge(Edge *edge) {
            assert(!concurrent); // only intended for use with non-concurrent RRT*
            edge_.store(edge);
        }

        bool casEdge(Edge*& expect, Edge* value, std::memory_order success, std::memory_order failure) {
            return edge_.compare_exchange_weak(expect, value, success, failure);
        }

        const State& state() const {
            return state_;
        }
    };

    template <typename State, typename Distance, typename Traj>
    class Node<State, Distance, Traj, false>
        : prrt_star::Edge<State, Distance, Traj, false>
    {
        using Edge = prrt_star::Edge<State, Distance, Traj, false>;
        friend class prrt_star::Edge<State, Distance, Traj, false>;

        State state_;
        bool goal_;

    public:
        // Needed? EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        template <typename ... Args>
        Node(bool goal, Args&& ... args)
            : state_(std::forward<Args>(args)...)
            , goal_(goal)
        {
        }

        template <typename ... Args>
        Node(Traj&& traj, Edge *parent, Distance cost, bool goal, Args&& ... args)
            : Edge(std::move(traj), parent, cost)
            , state_(std::forward<Args>(args)...)
            , goal_(goal)
        {
        }

        bool goal() const {
            return goal_;
        }

        const State& state() const {
            return state_;
        }

        Edge* edge(std::memory_order = std::memory_order_acquire) {
            return this;
        }

        const Edge* edge(std::memory_order = std::memory_order_acquire) const {
            return this;
        }
    };

    struct NodeKey {
        template <typename State, typename Distance, typename Traj, bool concurrent>
        const State& operator() (const Node<State, Distance, Traj, concurrent>* node) const {
            return node->state();
        }
    };

}

#endif
