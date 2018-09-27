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
#ifndef MPT_IMPL_CONDITION_TIMER_POSIX_HPP
#define MPT_IMPL_CONDITION_TIMER_POSIX_HPP

// DO NOT USE THIS FILE DIRECTLY, IT IS MEANT ONLY TO BE INCLUDED BY
// condition_timer.hpp WHEN MPT_USE_POSIX_TIMERS IS TRUE.

#include <atomic>
#include <signal.h>
#include <time.h>
#include "../log.hpp"

namespace unc::robotics::mpt::impl {
    class ConditionTimer {
        struct TimerRef {
            timer_t timerId_;
            std::atomic_bool done_{false};
            std::atomic_int refCount_{2};

            // we don't use constructor/destructors for RAII since we
            // have a gross initialization and cleanup process.  It
            // would be less gross if timer_t had a documented always
            // invalid value.
            //
            // if init fails, caller deletes
            // if timer_settime fails, caller calls cleanup()
            // release() is called once by either cancel() or timerDone()
            // release() is called a second time by the ~ConditionTimer
            // the second release() call will call cleanup()
            //
            // it is possible that timerDone() will be invoked after
            // ~ConditionTimer.  This is rare, but will not result in
            // a memory leak, since timerDone() will invoke the second
            // release() call.
            inline bool init(clock_t clockId) {
                struct sigevent sev;

                ::memset(&sev, 0, sizeof(sev));
                sev.sigev_notify = SIGEV_THREAD;
                sev.sigev_value.sival_ptr = static_cast<void*>(this);
                sev.sigev_notify_function = &ConditionTimer::timerDone;

                return (timer_create(clockId, &sev, &timerId_) == 0);
            }

            inline void cleanup() {
                if (timer_delete(timerId_) != 0)
                    MPT_LOG(WARN) << "failed to delete timer: " << errno;
                delete this;
            }

            inline void release() {
                if (--refCount_ == 0)
                    cleanup();
            }

            // cancels a pending timer.
            inline void cancel() {
                struct itimerspec tNew;
                struct itimerspec tOld;
                ::memset(&tNew, 0, sizeof(tNew));

                if (timer_settime(timerId_, 0, &tNew, &tOld) != 0) {
                    MPT_LOG(WARN) << "failed to cancel timer: " << errno;
                } else {
                    // if a pending timer has not yet fired, we need to
                    // release its reference as it will never fire now.
                    if (tOld.it_value.tv_sec != 0 || tOld.it_value.tv_nsec != 0)
                        release();
                }
            }
        };

        TimerRef *timerRef_{nullptr};

        inline static void timerDone(union sigval sv) {
            auto *ptr = reinterpret_cast<TimerRef*>(sv.sival_ptr);
            ptr->done_.store(true, std::memory_order_relaxed);
            ptr->release();
            // ptr may be invalid here
        }

        template <class Rep, class Period>
        TimerRef* createTimer(const std::chrono::duration<Rep, Period>& duration) {
            // TODO: select clockId based upon Clock type
            //
            // steady clocks => CLOCK_MONOTONIC
            // non-steady clocks => CLOCK_REALTIME
            //
            // see timer_create(2)
            clockid_t clockId = CLOCK_MONOTONIC;

            struct timespec now;
            if (clock_gettime(clockId, &now) != 0)
                throw std::system_error(errno, std::system_category(), "clock_gettime failed");

            TimerRef *ref = new TimerRef();
            if (!ref->init(clockId)) {
                delete ref;
                throw std::system_error(errno, std::system_category(), "timer_create failed");
            }

            struct itimerspec its;
            ::memset(&its, 0, sizeof(its));
            // auto s = std::chrono::time_point_cast<std::chrono::seconds>(endTime);
            // its.it_value.tv_sec = s.time_since_epoch().count();
            // its.it_value.tv_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - s).count();

            auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration);
            auto ns = now.tv_nsec + std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec).count();
            its.it_value.tv_sec = now.tv_sec + sec.count() + ns/1000000000L;
            its.it_value.tv_nsec = long(ns % 1000000000L);
            if (timer_settime(ref->timerId_, TIMER_ABSTIME, &its, nullptr) != 0) {
                ref->cleanup();
                throw std::system_error(errno, std::system_category(), "timer_settime failed");
            }

            // ref's count is now 2 (from constructor).  1 for the
            // ConditionTimer, and 1 for the timer queue.
            return ref;
        }

    public:
        // TODO: need to convert time_point to its structure using
        // appropriate clock_gettime clockid.
        //
        // template <class Clock, class Duration>
        // ConditionTimer(const std::chrono::time_point<Clock, Duration>& sleepTime)
        //     : timer_(createTimer(sleepTime))
        // {
        // }

        template <class Rep, class Period>
        ConditionTimer(const std::chrono::duration<Rep, Period>& duration)
            : timerRef_(createTimer(duration))
        {
        }

        ~ConditionTimer() {
            if (timerRef_) {
                // cancel the timer, if there is a pending timer event
                // then this will also release the reference from it.
                timerRef_->cancel();

                // release our reference
                timerRef_->release();
                // pter may be invalid here (cancel() call must come first)
            }
        }

        decltype(auto) doneFn() {
            return [done = &timerRef_->done_] { return done->load(std::memory_order_relaxed); };
        }

        template <typename Pred>
        decltype(auto) doneFn(Pred& pred) {
            return [done = &timerRef_->done_, &pred] {
                if (done->load(std::memory_order_relaxed))
                    return true;
                if (!pred())
                    return false;
                done->store(true, std::memory_order_relaxed);
                return true;
            };
        }
    };
}

#endif
