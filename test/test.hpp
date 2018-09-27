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

#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <sstream>
#include <vector>
#include <exception>

int main(int argc, char *argv[]);

template <typename _T>
struct Approx {
    _T value_;
    unsigned ulps_;

    Approx(_T value, unsigned ulps = 4) : value_(value), ulps_(ulps) {}

    operator _T () const { return value_; }
};

template <typename _T>
bool operator == (_T a, const Approx<_T>& b) {
    _T diff = std::abs(a - b.value_);
    return diff <= std::numeric_limits<_T>::epsilon() * std::abs(b.value_) * b.ulps_
        || diff < std::numeric_limits<_T>::min();
}

namespace test {

    static std::atomic_uintmax_t g_assertionCount{0};

    class TestFailedException : public std::exception {
        std::string what_;

    public:
        TestFailedException(const std::string& what) : what_(what) {}
        virtual const char* what() const noexcept override { return what_.c_str(); }
    };

    namespace detail {
        template <typename _T, class _Enable = void>
        struct Format {
            const _T& value_;
            Format(const _T& value) : value_(value) {}

            template <typename _Char, typename _Traits = std::char_traits<_Char>>
            friend auto& operator << (std::basic_ostream<_Char, _Traits>& out, const Format& obj) {
                return out << obj.value_;
            }
        };

        template <typename _T>
        struct Format<_T, std::enable_if_t<std::is_floating_point_v<_T>>> {
            _T value_;
            Format(_T value) : value_(value) {}

            template <typename _Char, typename _Traits = std::char_traits<_Char>>
            friend auto& operator << (std::basic_ostream<_Char, _Traits>& out, const Format& obj) {
                out.precision(std::numeric_limits<_T>::max_digits10);
                return out << obj.value_;
            }
        };

        template <>
        struct Format<bool, void> {
            bool value_;
            Format(bool value) : value_(value) {}

            template <typename _Char, typename _Traits = std::char_traits<_Char>>
            friend auto& operator << (std::basic_ostream<_Char, _Traits>& out, const Format& obj) {
                out.setf(std::ios_base::boolalpha);
                return out << obj.value_;
            }
        };
    }

    template <typename _T>
    class Expect {
        _T value_;

        const char *expr_;
        const char *file_;
        int line_;

        mutable bool checked_{false};

        template <typename _E>
        void fail(const char *op, const _E& expect) const {
            std::ostringstream str;
            using detail::Format;
            str << "Expected " << expr_ << op << Format<_T>(expect) //static_cast<const _T>(expect)
                << ", got " << Format<_T>(value_) << " at " << file_ << ':' << line_;
            throw TestFailedException(str.str());
        }

    public:
        Expect(const _T& value, const char *expr, const char *file, int line)
            : value_(value), expr_(expr), file_(file), line_(line)
        {
        }

        Expect(_T&& value, const char *expr, const char *file, int line)
            : value_(std::move(value)), expr_(expr), file_(file), line_(line)
        {
        }

        ~Expect() {
            assert(checked_);
        }

#define OP(_op_) \
        template <typename _E>                          \
        void operator _op_ (const _E& expect) const {   \
            assert(!checked_);                          \
            checked_ = true;                            \
            if (!(value_ _op_ expect))                  \
                fail(" " #_op_ " ", expect);            \
            ++g_assertionCount;                         \
        }

        OP(==)
        OP(!=)
        OP(<)
        OP(>)
        OP(<=)
        OP(>=)
#undef OP
    };

    // specialization of std::atomic<_T> handles the problem in which
    // std::atomic is not copiable which causes the constructor to not
    // compile.  Here we have an atomic extend the expectation for the
    // contained type which cause the value in the atomic to be
    // copied instead of the atomic.
    template <typename _T>
    class Expect<std::atomic<_T>> : public Expect<_T> {
    public:
        using Expect<_T>::Expect;
    };

    class TestCase {
        static auto& testCases() {
            static std::vector<TestCase*> list;
            return list;
        }

        std::string name_;
    public:
        TestCase(const std::string& name) : name_(name) {
            testCases().push_back(this);
        }
        virtual ~TestCase() {}
        virtual void run_test_case() = 0;
        friend int ::main(int, char*argv[]);
    };
}

#define EXPECT(expr) (::test::Expect<typename std::decay<decltype(expr)>::type>( \
                          expr, #expr, __FILE__, __LINE__))

#define TEST(name)                                              \
    struct test_case_ ## name : public ::test::TestCase {       \
        test_case_ ## name () : TestCase(#name) {}              \
        virtual void run_test_case() override;                  \
    };                                                          \
    test_case_ ## name test_instance_ ## name;                  \
    void test_case_ ## name :: run_test_case()


int main(int argc, char *argv[]) {
    using namespace test;

    std::size_t passed = 0;
    std::size_t testCount = 0;

    for (TestCase *test : TestCase::testCases()) {
        ++testCount;
        auto assertionsBefore = g_assertionCount.load();
        try {
            auto start = std::chrono::high_resolution_clock::now();
            test->run_test_case();
            auto elapsed = std::chrono::high_resolution_clock::now() - start;
            auto nAsserts = g_assertionCount.load() - assertionsBefore;
            std::ostringstream msg;
            //msg.imbue(std::locale(""));
            msg << test->name_ << " \33[32mpassed ✓\33[0m ("
                << (nAsserts ? "" : "\33[31m")
                << nAsserts << " assertion" << (nAsserts == 1 ? "" : "s")
                << (nAsserts ? "" : "\33[0m")
                << ", " << std::chrono::duration<double, std::milli>(elapsed).count()
                << " ms)\n";
            std::cout << msg.str() << std::flush;
            ++passed;
        } catch (const std::exception& e) {
            auto nAsserts = g_assertionCount.load() - assertionsBefore;
            std::ostringstream msg;
            //msg.imbue(std::locale(""));
            msg << test->name_ << " \33[31;1mfailed ⚠\33[0m  after "
                << nAsserts << " assertion" << (nAsserts == 1 ? "" : "s") << "\n\t"
                << e.what() << "\n";
            std::cout << msg.str() << std::flush;
        } catch (...) {
            std::cout << "Uncaught exception" << std::endl;
        }
    }

    std::cout << passed << " of "
              << testCount << " test" << (testCount == 1?"":"s") << " passed."
              << std::endl;

    return !(testCount && testCount == passed);
}

// In lldb use:
//   break set -E C++
// to break when any exception is thrown, or:
//   break set -F test::TestFailedException
// to break when a test fails.
