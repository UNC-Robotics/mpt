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
#ifndef MPT_IMPL_PPRM_COMPONENT_HPP
#define MPT_IMPL_PPRM_COMPONENT_HPP

#include <atomic>

namespace unc::robotics::mpt::impl::pprm {
    struct Component {
        enum Flags : unsigned char {
            kNone = 0,
            kStart = 1,
            kGoal = 2,
            kSolution = kStart | kGoal
        };

        std::size_t size_;
        Flags flags_;
        std::atomic<Component*> next_{nullptr};

    public:
        inline Component(std::size_t size, Flags flags)
            : size_(size), flags_(flags)
        {
        }

        inline Component(Component *a, Component *b) {
            update(a, b);
        }

        inline std::size_t size() const {
            return size_;
        }

        inline void update(Component *a, Component *b) {
            size_ = a->size_ + b->size_;
            flags_ = static_cast<Flags>(a->flags_ | b->flags_);
        }

        inline Component *next() {
            Component *next = next_.load(std::memory_order_acquire);
            if (next != nullptr) {
                if (Component *n = next->next()) {
                    next_.compare_exchange_strong(next, n, std::memory_order_relaxed);
                    return n;
                }
            }
            return next;
        }

        inline bool casNext(Component*& expect, Component *value, std::memory_order order) {
            return next_.compare_exchange_strong(expect, value, order);
        }

        inline bool isGoal() {
            return (flags_ & kGoal) != 0;
        }

        inline bool isStart() {
            return (flags_ & kStart) != 0;
        }

        inline bool isSolution() {
            return flags_ == kSolution;
        }
    };
}

#endif
