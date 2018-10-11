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
#ifndef MPT_IMPL_PPRM_IRS_NODE_BASE_HPP
#define MPT_IMPL_PPRM_IRS_NODE_BASE_HPP

#include "edge.hpp"
#include <atomic>

namespace unc::robotics::mpt::impl::pprm_irs {
    template <typename State, typename Distance, bool keepDense>
    class NodeBase {
        using Edge = pprm_irs::Edge<State, Distance, keepDense>;
        State state_;
        
        std::atomic<Component*> component_;
        std::atomic<Edge*> sparseHead_{nullptr};

    public:
        template <typename ... Args>
        NodeBase(Component *component, Args&& ... args)
            : state_(std::forward<Args>(args)...)
            , component_(component)
        {
        }

        const State& state() const {
            return state_;
        }

        const Edge *sparseHead(std::memory_order order) const {
            return sparseHead_.load(order);
        }

        void addSparseEdge(Edge *edge) {
            linkEdge(sparseHead_, edge);
        }

        Component *component() {
            Component *c = component_.load(std::memory_order_relaxed);
            if (Component *n = c->next()) {
                component_.compare_exchange_strong(c, n, std::memory_order_relaxed);
                return n;
            }
            return c;
        }
    };

    template <typename State, typename Distance, bool keepDense>
    class Node;

    template <typename State, typename Distance>
    class Node<State, Distance, false> : public NodeBase<State, Distance, false> {
    public:
        using NodeBase<State, Distance, false>::NodeBase;
    };

    template <typename State, typename Distance>
    class Node<State, Distance, true> : public NodeBase<State, Distance, true> {
        using Edge = pprm_irs::Edge<State, Distance, true>;
        
        std::atomic<Edge*> denseHead_{nullptr};
        
    public:
        using NodeBase<State, Distance, true>::NodeBase;

        void addDenseEdge(Edge *edge) {
            linkEdge(denseHead_, edge);
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
