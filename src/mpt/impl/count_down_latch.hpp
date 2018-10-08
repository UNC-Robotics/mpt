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
#ifndef MPT_IMPL_COUNT_DOWN_LATCH_HPP
#define MPT_IMPL_COUNT_DOWN_LATCH_HPP

#include <atomic>
#include <mutex>
#include <condition_variable>

namespace unc::robotics::mpt::impl {

    // Synchronization primitive which blocks the wait() method until
    // a pre-specified number of countDown() calls are made.
    class CountDownLatch {
        std::atomic<int> count_;

        // unfortunately this seems to be the method to have one
        // thread sleep and be awaken by the other.  We don't really
        // need a mutex since our state fits in an atomic, but
        // condition variables require a mutex.
        mutable std::mutex mutex_;
        mutable std::condition_variable cv_;
        
    public:
        inline CountDownLatch(int n)
            : count_(n)
        {
            assert(n >= 0);
        }

        inline void countDown() {
            int before = count_.fetch_sub(1, std::memory_order_relaxed);

            // fetch_sub returns the value before subtraction, if we
            // already reached 0, then this is an extra call to
            // countDown().
            assert(before != 0);
            
            if (before == 1)
                cv_.notify_all();
        }

        inline void wait() const {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return count_.load(std::memory_order_relaxed) == 0; });
        }
    };
}

#endif
