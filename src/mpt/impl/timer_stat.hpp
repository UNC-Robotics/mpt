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
#ifndef MPT_IMPL_TIMER_STAT_HPP
#define MPT_IMPL_TIMER_STAT_HPP

#include "../log.hpp"
#include <chrono>

namespace unc::robotics::mpt::impl {
    template <typename Clock>
    std::pair<typename Clock::duration,unsigned> computeClockOverhead(
        typename Clock::duration loopTime = std::chrono::milliseconds(1))
    {
        typename Clock::duration elapsed;
        unsigned count = 1;
        typename Clock::time_point start = Clock::now();
        do {
            ++count;
        } while ((elapsed = (Clock::now() - start)) < loopTime);

        MPT_LOG(TRACE) << log::type_name<Clock>() << "::now() overhead = " << typename Clock::duration(elapsed.count() / count);
        return std::make_pair(elapsed, count);
    }

    template <typename Clock>
    std::pair<typename Clock::duration, unsigned> clockOverhead() {
        static const std::pair<typename Clock::duration, unsigned> overhead = computeClockOverhead<Clock>();
        return overhead;
    }
    
    template <typename C = std::chrono::steady_clock>
    class TimerStat {
    public:
        using Clock = C;
        using Duration = typename Clock::duration;
        using TimePoint = typename Clock::time_point;

    private:
        Duration elapsed_{};
        unsigned count_{0};

    public:
        TimePoint start() const {
            return Clock::now();
        }

        TimerStat& operator += (const TimerStat& other) {
            elapsed_ += other.elapsed_;
            count_ += other.count_;
            return *this;
        }

        TimerStat& operator += (Duration duration) {
            elapsed_ += duration;
            ++count_;
            return *this;
        }

        TimerStat& operator += (TimePoint& start) {
            TimePoint now = Clock::now();
            elapsed_ += now - start;
            ++count_;
            start = now;
            return *this;
        }

        Duration elapsed() const {
            return elapsed_;
        }

        unsigned count() const {
            return count_;
        }

        Duration average() const {
            return Duration(elapsed_.count() / count_);
        }

        friend decltype(auto) operator << (log::Event& evt, const TimerStat& stat) {
            if (stat.count() == 0)
                return evt << "0 s over 0 calls (avarage NaN s)";

            // Assumption, Clock::now() has a fixed overhead that can
            // be broken down into overhead before the time point and
            // overhead after the time point, we get:
            //
            //    overhead = pre + post
            //
            // Each timed call is thus:
            //
            //    timed_with_overhead = overhead + measure + overhead
            //         = pre + post + measure + pre + post
            //
            // Rewriting, noting that the time point is measured
            // between the pre and post, we get:
            //
            //         = pre + (post + measure + pre) + post
            //         = (overhead + measure) + overhead
            //
            // And we argue that the actual time consumed by the
            // measured call is timed duration - 1*overhead.  Whereas
            // the whole system is slowed down by 2*overhead.

            Duration overhead{stat.count() * clockOverhead<Clock>().first.count()
                / clockOverhead<Clock>().second};

            Duration elapsed = stat.elapsed() - overhead;

            return evt << elapsed << " over "
                       << stat.count() << " calls (overhead "
                       << (overhead+overhead) << ")";
        }
    };

    // TimerStat placeholder specialization for when something does
    // not wish to spend time tracking stats.
    template <>
    class TimerStat<void> {
    public:
        static TimerStat<void>& instance() {
            static TimerStat<void> t;
            return t;
        }
    };

    // Specialized base class for Timer.  The specialization allows
    // for void timers to carry no overhead.
    template <typename Stat>
    class TimerImpl;

    template <typename Clock>
    class TimerImpl<TimerStat<Clock>> {
        using Stat = TimerStat<Clock>;
        
        Stat& stat_;
        typename Clock::time_point start_{Clock::now()};

    public:
        TimerImpl(Stat& stat)
            : stat_(stat)
        {
        }

        ~TimerImpl() {
            stat_ += elapsed();
        }

        typename Clock::duration elapsed() const {
            return Clock::now() - start_;
        }
    };

    template <>
    class TimerImpl<TimerStat<void>> {
    public:
        TimerImpl(TimerStat<void>) {}
    };


    // Timer provides RAII for timing a code block's execution time.
    template <typename Stat>
    class Timer : public TimerImpl<Stat> {
        using Base = TimerImpl<Stat>;
    public:
        Timer(Stat& stat) : Base(stat) {}
    };
}

#endif
