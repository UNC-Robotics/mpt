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
#ifndef MPT_IMPL_CONDITION_TIMER_HPP
#define MPT_IMPL_CONDITION_TIMER_HPP

#include "../log.hpp"
#include "finally.hpp"
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

// Using POSIX timers requires -lrt on linux, and it does not appear
// to have much of an advantage over std::thread +
// std::condition_variable.
#if MPT_USE_POSIX_TIMERS && _POSIX_C_SOURCE >= 199309L
#include "condition_timer_posix.hpp"
#else
namespace unc::robotics::mpt::impl {
    class ConditionTimer {
        std::atomic_bool done_{false};
        std::mutex mutex_;
        std::condition_variable cv_;
        std::thread thread_;

        template <class Clock, class Duration>
        void sleepUntil(const std::chrono::time_point<Clock, Duration>& timeoutTime) {
            Finally finally([&] { done_.store(true, std::memory_order_relaxed); });
            std::unique_lock<std::mutex> lock(mutex_);
            try {
                // wait_until is pthread_cond_timedwait when using
                // pthreads--this is good, since it means the thread
                // will be idle until the target time is reached or
                // until notified (under normal conditions).
                // Unfortunately, there does not seem to be a
                // guarantee that this will use a steady clock, which
                // means we that we cannot count on changes to the
                // system clock causing variations in actual
                // wall-clock time elapsed.
                while (!done_.load(std::memory_order_relaxed))
                    if (cv_.wait_until(lock, timeoutTime) == std::cv_status::timeout)
                        break;
            } catch (const std::system_error& err) {
                MPT_LOG(WARN) << "timer failed: " << err.what();
            }
        }

        inline void setDone() {
            bool expect = false;
            if (done_.compare_exchange_strong(expect, true, std::memory_order_relaxed))
                cv_.notify_one();
        }

    public:
        template <class Clock, class Duration>
        ConditionTimer(const std::chrono::time_point<Clock, Duration>& sleepTime)
            : thread_(&ConditionTimer::sleepUntil<Clock,Duration>, this, sleepTime)
        {
        }

        template <class Rep, class Period>
        ConditionTimer(const std::chrono::duration<Rep, Period>& duration)
            : ConditionTimer(std::chrono::system_clock::now() + duration)
        {
        }

        inline ~ConditionTimer() {
            setDone();
            thread_.join();
        }

        // Returns a lambda that will return true once the timer's
        // elapsed time has passed.
        inline decltype(auto) doneFn() {
            return [done = &done_] {
                return done->load(std::memory_order_relaxed);
            };
        }

        // Returns a lambda that will return true once the timer's
        // elapsed time has passed or the predicate argument returns
        // true (i.e., which ever happens first).
        template <typename Pred>
        decltype(auto) doneFn(Pred& pred) {
            return [this, &pred] {
                if (done_.load(std::memory_order_relaxed))
                    return true;
                if (!pred())
                    return false;
                setDone();
                return true;
            };
        }
    };
}
#endif
#endif
