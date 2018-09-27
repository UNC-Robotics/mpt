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

#ifndef MPT_LOG_HPP_
#define MPT_LOG_HPP_

//#include "impl/nocopy_stringstream.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <unistd.h>
#include <Eigen/Dense>

namespace unc::robotics::mpt::log::detail {
    template <typename Ratio> struct si_prefix;
    template <> struct si_prefix<std::ratio<1>> {
        static constexpr const char *value() { return ""; }
    };
    template <> struct si_prefix<std::milli> {
        static constexpr const char *value() { return "m"; }
    };
    template <> struct si_prefix<std::micro> {
        static constexpr const char *value() { return "u"; }
    };
    template <> struct si_prefix<std::nano> {
        static constexpr const char *value() { return "n"; }
    };

    template <typename T> struct is_tuple : std::false_type {};
    template <typename ... U> struct is_tuple<std::tuple<U...>> : std::true_type {};
    template <typename T> struct is_tuple<const T> : is_tuple<T> {};
    template <typename T> struct is_tuple<volatile T> : is_tuple<T> {};
    template <typename T> struct is_tuple<T&> : is_tuple<T> {};
    template <typename T> struct is_tuple<T&&> : is_tuple<T> {};

    template <typename T> struct is_duration : std::false_type {};
    template <class R, class P> struct is_duration<std::chrono::duration<R, P>> : std::true_type {};

    template <std::intmax_t value>
    constexpr std::intmax_t log10 = (value > 1 ? log10<value/10> + 1 : 0);

    class ThreadName {
        std::string name_;

        static std::string autoName() {
            static std::atomic<std::size_t> count{};
            std::ostringstream tmp;
            tmp << std::hex << std::setfill('0') << std::setw(4) << count++;
            // return "thread-" + std::to_string(++count);
            return tmp.str();
            // std::ostringstream tmp;
            // tmp << std::hex << std::this_thread::get_id();
            // return tmp.str();
        }

    public:
        ThreadName()
            : name_(autoName())
        {
        }

        static ThreadName& instance() {
            thread_local ThreadName x;
            return x;
        }

        const std::string& get() const {
            return name_;
        }

        void set(const std::string& name) {
            name_ = name;
        }
    };

    // Interesting idea to help out logf by forcing the format to
    // be a literal and allowing for compile-time information such
    // as the string length.
    //
    // class ConstString {
    //     const char *ptr_;
    //     std::size_t size_;
    // public:
    //     template <std::size_t _N>
    //     ConstString(const char(&ptr)[_N]) : ptr_(ptr), size_(_N - 1) {}

    //     constexpr char operator[] (std::size_t n) const {
    //         return n < size_ ? ptr_[n] : throw std::out_of_range("");
    //     }

    //     constexpr std::size_t size() const {
    //         return size_;
    //     }

    //     operator const char * () const {
    //         return ptr_;
    //     }
    // };

    class LogQueue {
        struct Item {
            std::string msg_;
            Item *next_;
            inline Item(std::string&& msg, Item *next) : msg_(std::move(msg)), next_(next) {}
        };

        struct Lock {
            std::atomic_bool& lock_;
            bool ticket_;
            inline Lock(std::atomic_bool& lock) : lock_(lock), ticket_{lock.exchange(true)} {}
            inline ~Lock() { if (!ticket_) lock_.store(false, std::memory_order_release); }
            inline operator bool() { return !ticket_; }
        };

        // Here we check if stderr is a terminal or not.  If it is a
        // terminal, we write ANSI color codes, otherwise we don't.
        // Note that std::clog is documented to write to stderr.
        bool isTTY_{::isatty(STDERR_FILENO) == 1};

        std::atomic_bool locked_{false};
        std::atomic<Item*> list_{nullptr};

        inline Item *dequeue() {
            // log messages are placed in LIFO order, here we reverse
            // it to make sure we output in FIFO order.
            Item *prev = nullptr;
            for (Item *head = list_.exchange(nullptr), *next ; head != nullptr ; head = next) {
                next = head->next_;
                head->next_ = prev;
                prev = head;
            }
            return prev;
        }

        inline void drain() {
            for (Item *item = dequeue(), *next ; item ; item = next) {
                next = item->next_;
                std::clog << item->msg_;
                delete item;
            }
        }

    public:
        // LogQueue() {
        //     std::clog.sync_with_stdio(false);
        // }

        inline bool isTTY() const {
            return isTTY_;
        }

        inline ~LogQueue() {
            // destructor assumes no concurrent access.  Thus we do
            // not need to lock, and indeed the lock should not be
            // held.
            assert(locked_ == false);
            drain();
            if (isTTY_)
                std::clog << "\33[90m[log queue drained] (" << __FILE__ << ")\33[0m\n" << std::flush;
        }

        inline void operator() (std::ostringstream& msg) {
            if (isTTY_) {
                msg << "\33[0m\n";
            } else {
                msg << '\n';
            }

            (*this)(msg.str());
        }

        inline void operator() (std::string&& str) {
            // The queue operator attempts to output the string as
            // soon as possible.  However, to avoid processing delays,
            // if the stream is currently locked, we append the
            // message to a lock-free queue and return.
            if (Lock lock{locked_}) {
                drain();
                std::clog << str << std::flush;
            } else {
                Item *item = new Item(std::move(str), list_.load(std::memory_order_acquire));
                while (!list_.compare_exchange_weak(item->next_, item, std::memory_order_acq_rel))
                    ;

                if (Lock lock{locked_})
                    drain();
            }
        }
    };
}

namespace unc::robotics::mpt::log::level {
    struct ALL {};

#define MPT_LOG_DEFINE_LEVEL(NAME, NEXT, COLOR)                         \
    struct NAME {                                                       \
        typedef NEXT next;                                              \
        static constexpr const char *name() { return #NAME; }           \
        static constexpr const char *color() {                          \
            return "\33[" COLOR "m";                                    \
        }                                                               \
    }

    MPT_LOG_DEFINE_LEVEL(TRACE, ALL, "34"); // blue
    MPT_LOG_DEFINE_LEVEL(DEBUG, TRACE, "36"); // cyan
    MPT_LOG_DEFINE_LEVEL(INFO , DEBUG,  "32");  // green
    MPT_LOG_DEFINE_LEVEL(WARN , INFO, "33");  // yellow
    MPT_LOG_DEFINE_LEVEL(ERROR, WARN, "31;1"); // red, bold
    MPT_LOG_DEFINE_LEVEL(FATAL, ERROR,  "101;30");  // background red, black text
#undef MPT_LOG_DEFINE_LEVEL

    struct NONE {
        typedef FATAL next;
    };
}

// At compile time, define MPT_LOG_LEVEL to the level logging level to
// one of: ALL, TRACE, DEBUG, INFO, WARN, ERROR, FATAL or NONE.  NONE
// should be avoided as it hides FATAL messages too.  ALL is an alias
// for the lowest level (TRACE).  If MPT_LOG_LEVEL is not defined, the
// log level is determined by whether or not NDEBUG is defined.
#ifndef MPT_LOG_LEVEL
  #ifdef NDEBUG
    #define MPT_LOG_LEVEL INFO
  #else
    #define MPT_LOG_LEVEL ALL
  #endif
#endif


namespace unc::robotics::mpt::log {

    template <typename Ratio = std::milli>
    class Timer {
        using Clock = std::chrono::high_resolution_clock;
        Clock::time_point start_{Clock::now()};
    public:
        double elapsed() const {
            return std::chrono::duration<double, Ratio>(Clock::now() - start_).count();
        }
    };

    template <typename Level, class = std::void_t<>>
    struct is_enabled : std::false_type {};
    template <typename Level>
    struct is_enabled<Level, std::void_t<typename Level::next>>  : is_enabled<typename Level::next> {};
    template <>
    struct is_enabled<level:: MPT_LOG_LEVEL> : std::true_type {};

    inline void setThreadName(const std::string& name) {
        detail::ThreadName::instance().set(name);
    }

    inline const std::string& getThreadName() {
        return detail::ThreadName::instance().get();
    }

    class Event {
        std::ostringstream msg_;
        // impl::NoCopyStringOutputStream<char> msg_;

        inline static detail::LogQueue& logQueue() {
            static detail::LogQueue instance;
            return instance;
        }

    public:
        template <typename Level>
        Event(const Level&, const char *file, int line) {
            using namespace std::chrono;
            auto now = system_clock::now();
            std::time_t t = system_clock::to_time_t(now);
            auto millis = duration_cast<milliseconds>(
                now.time_since_epoch()).count() % 1000;

            // 12:34:56.789 INFO  [main] (file.hpp:123) message goes here
            if (logQueue().isTTY()) {
                // use the hash value of the thread id to color code
                // (using the 256 color palette) the thread.  Colors in
                // the range [0..15] come from the 4-bit color palette.
                // Colors in the range [232..255] are grayscale.  We
                // choose a color in the range [16..231].
                std::hash<std::string> hasher;
                std::size_t th = hasher(getThreadName()) % 216 + 16;

                msg_ << std::put_time(std::localtime(&t), "%T") << '.'
                     << std::setfill('0') << std::setw(3) << millis << ' '
                     << Level::color()
                     << std::setfill(' ') << std::left << std::setw(5) << Level::name()
                     << "\33[0m [\33[38;5;" << th << "m" << getThreadName() << "\33[0m] \33[37m("
                     << file << ':' << line << ")\33[0m "
                     << Level::color();
            } else {
                msg_ << std::put_time(std::localtime(&t), "%T") << '.'
                     << std::setfill('0') << std::setw(3) << millis << ' '
                     << std::setfill(' ') << std::left << std::setw(5) << Level::name()
                     << " [" << getThreadName() << "] ("
                     << file << ':' << line << ") ";
            }
        }

        inline ~Event() {
            logQueue()(msg_);
        }

        template <typename U>
        std::enable_if_t<
            !std::is_base_of_v<Eigen::DenseBase<std::decay_t<U>>, std::decay_t<U>> &&
            !std::is_base_of_v<Eigen::QuaternionBase<std::decay_t<U>>, std::decay_t<U>> &&
            !detail::is_duration<std::decay_t<U>>::value &&
            !detail::is_tuple<U>::value, Event&>
        operator << (const U& arg) {
            msg_ << arg;
            return *this;
        }

        template <typename Ratio>
        Event& operator << (const Timer<Ratio>& t) {
            msg_ << t.elapsed() << " " << detail::si_prefix<Ratio>::value() << "s";
            return *this;
        }

        template <typename ... U>
        Event& operator << (const std::tuple<U...>& arg) {
            write(arg, std::index_sequence_for<U...>{});
            return *this;
        }

        template <typename Derived>
        Event& operator << (const Eigen::QuaternionBase<Derived>& q) {
            Eigen::IOFormat fmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ", "", "", "[", "]");
            msg_ << q.coeffs().format(fmt);
            return *this;
        }

        template <typename Derived>
        Event& operator << (const Eigen::DenseBase<Derived>& m) {
            Eigen::IOFormat fmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ", "", "", "[", "]");
            if (m.cols() == 1 && m.rows() > 1) {
                msg_ << m.transpose().format(fmt) << "^T";
            } else {
                msg_ << m.format(fmt);
            }
            return *this;
        }

        template <typename Rep, typename Period>
        Event& operator << (const std::chrono::duration<Rep, Period>& d) {
            if constexpr (std::is_integral_v<Rep> && Period::num == 1 && Period::den % 10 == 0 && Period::den > 1) {
                // output directly as an integer if easy to do.  This
                // route keeps precision, and may be faster.
                msg_ << d.count() / static_cast<Rep>(Period::den) << '.';
                auto oldFill = msg_.fill('0');
                auto oldWidth = msg_.width(detail::log10<Period::den>);
                auto oldPos = msg_.setf(std::ios_base::right, std::ios_base::adjustfield);
                msg_ << d.count() % Period::den;
                msg_.fill(oldFill);
                msg_.width(oldWidth);
                msg_.setf(oldPos, std::ios_base::adjustfield);
                msg_ << " s";
            } else {
                // otherwise convert to double and output.  This
                // typically loses some precision (mostly due to
                // formatting, but also after 104 days with
                // 64-bit nanoseconds)
                msg_ << std::chrono::duration<double>(d).count() << " s";
            }
            return *this;
        }

        // This is here to allow the expansion of a conditional log
        // without using an if/else.  See MPT_IMPL_LOG_S.  Essentially
        // it expands to:
        //    enabledBool && Event(...) << "message" << ...;
        // Which is parsed as:
        //    enabledBool && (Event(...) << "message" << ... );
        inline constexpr operator bool () const { return false; }

        // TODO:
        // template <typename Scalar, int rows>
        // Event& operator << (const Eigen::Matrix<Scalar, rows, 1>& m) {
        //     msg_ << m.transpose() << "^T";
        //     return *this;
        // }

    private:
        template <std::size_t I, typename T>
        void ith(const T& t) {
            if (I) msg_ << ", ";
            // msg_ << '(';
            *this << std::get<I>(t);
            // msg_ << ')';
        }

        template <typename T, std::size_t ... I>
        void write(const T& t, std::index_sequence<I...>) {
            msg_ << '{';
            (ith<I>(t), ...);
            msg_ << '}';
        }
    };

    template <typename Level>
    __attribute__((format(printf, 3, 4)))
    int logf(const char *file, int line, const char *fmt, ...) {
        Event evt(Level{}, file, line);
        va_list args1;
        va_list args2;
        va_start(args1, fmt);
        va_copy(args2, args1);
        int n = std::vsnprintf(nullptr, 0, fmt, args1);
        va_end(args1);
        if (n >= 0) {
            std::vector<char> v(n+1);
            n = std::vsnprintf(v.data(), v.size(), fmt, args2);
            evt << static_cast<const char*>(v.data());
        }
        va_end(args2);
        return n;
    }

    template <class T>
    constexpr std::string_view type_name() {
        // https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c
#ifdef __clang__
        // "std::string_view unc::robotics::mpt::log::type_name() [T = XYZ]"
        std::string_view p = __PRETTY_FUNCTION__;
        return std::string_view(p.data() + 59, p.size() - 59 - 1);
#elif defined(__GNUC__)
        // "constexpr std::string_view unc::robotics::mpt::log::type_name() [with T = XYZ; std::string_view = std::basic_string_view<char>]"
        //
        // the stackoverflow answer uses p.find(';', 74) to trim out
        // the definition of std::string_view.  here we just assume
        // that std::string_view will not change, and if it does, that
        // 74 may also become invalid.
        std::string_view p = __PRETTY_FUNCTION__;
        return std::string_view(p.data() + 74, p.size() - 74 - 50);
#elif defined(_MSC_VER)
        // this is untested (as is everything MSC code) :(
        std::string_view p = __FUNCSIG__;
        return std::string_view(p.data() + 109, p.size() - 109 - 7);
#endif
    }
}

// _S for 'stream'
// MPT_LOG(TRACE) << "messages" << data << etc;
#define MPT_IMPL_LOG_S(LVL)                                             \
    ::unc::robotics::mpt::log::is_enabled< ::unc::robotics::mpt::log::level::LVL >::value && \
    ::unc::robotics::mpt::log::Event( ::unc::robotics::mpt::log::level::LVL {}, __FILE__, __LINE__)

// _M for 'message' (single)
// MPT_LOG(TRACE, "text")
#define MPT_IMPL_LOG_M(LVL, msg) do {                                   \
        if (::unc::robotics::mpt::log::is_enabled< ::unc::robotics::mpt::log::level::LVL >::value) \
            ::unc::robotics::mpt::log::logf< ::unc::robotics::mpt::log::level::LVL >(__FILE__, __LINE__, "%s", msg); \
    } while (false)

// _F for formatted
// MPT_LOG(TRACE, "printf %s formatting, with up to %d args", "style", 58)
#define MPT_IMPL_LOG_F(LVL, ...) do {                                   \
        if (::unc::robotics::mpt::log::is_enabled< ::unc::robotics::mpt::log::level::LVL >::value) \
            ::unc::robotics::mpt::log::logf< ::unc::robotics::mpt::log::level::LVL >(__FILE__, __LINE__, __VA_ARGS__); \
    } while (false)

// _MPT_SELECT
#define MPT_IMPL_SELECT(                                                \
    PREFIX,                                                             \
    a60, a59, a58, a57, a56, a55, a54, a53, a52, a51,                   \
    a50, a49, a48, a47, a46, a45, a44, a43, a42, a41,                   \
    a40, a39, a38, a37, a36, a35, a34, a33, a32, a31,                   \
    a30, a29, a28, a27, a26, a25, a24, a23, a22, a21,                   \
    a20, a19, a18, a17, a16, a15, a14, a13, a12, a11,                   \
    a10, a09, a08, a07, a06, a05, a04, a03, a02, a01,                   \
    SUFFIX,...) MPT_IMPL_ ## PREFIX ## _ ## SUFFIX

// Usage:
//
// MPT_LOG(LEVEL, "message")
// MPT_LOG(LEVEL, "printf-format", args...)
// MPT_LOG(LEVEL) << "stream message" << ...;
//
// Where LEVEL is one of:
//   TRACE, DEBUG, INFO, WARN, ERROR, FATAL
#define MPT_LOG(...) MPT_IMPL_SELECT(                                   \
        LOG, __VA_ARGS__,                                               \
        F,F,F,F,F, F,F,F,F,F, F,F,F,F,F, F,F,F,F,F,                     \
        F,F,F,F,F, F,F,F,F,F, F,F,F,F,F, F,F,F,F,F,                     \
        F,F,F,F,F, F,F,F,F,F, F,F,F,F,F, F,F,F,M,S,)(__VA_ARGS__)


#endif // MPT_LOG_HPP_
