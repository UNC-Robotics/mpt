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
    template <typename State, typename Distance, bool concurrent>
    class Link;

    template <typename State, typename Distance, bool concurrent>
    class Node {
        using Link = prrt_star::Link<State, Distance, concurrent>;

        State state_;
        Atom<Link*, concurrent> link_{nullptr};
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

        Link* link(std::memory_order order) {
            return link_.load(order);
        }

        const Link* link(std::memory_order order) const {
            return link_.load(order);
        }

        void setLink(Link *link) {
            assert(!concurrent); // only intended for use with non-concurrent RRT*
            link_.store(link);
        }

        bool casLink(Link*& expect, Link* value, std::memory_order success, std::memory_order failure) {
            return link_.compare_exchange_weak(expect, value, success, failure);
        }

        const State& state() const {
            return state_;
        }
    };

    template <typename State, typename Distance>
    class Node<State, Distance, false>
        : prrt_star::Link<State, Distance, false>
    {
        using Link = prrt_star::Link<State, Distance, false>;
        friend class prrt_star::Link<State, Distance, false>;

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
        Node(Link *parent, Distance cost, bool goal, Args&& ... args)
            : Link(parent, cost)
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

        Link* link(std::memory_order = std::memory_order_acquire) {
            return this;
        }

        const Link* link(std::memory_order = std::memory_order_acquire) const {
            return this;
        }
    };

    struct NodeKey {
        template <typename State, typename Distance, bool concurrent>
        const State& operator() (const Node<State, Distance, concurrent>* node) const {
            return node->state();
        }
    };

}

#endif
