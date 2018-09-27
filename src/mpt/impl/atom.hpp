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
#ifndef MPT_IMPL_ATOM_HPP
#define MPT_IMPL_ATOM_HPP

#include <atomic>
#include <cassert>
#include <type_traits>

namespace unc::robotics::mpt::impl {
    template <typename T>
    class NonAtomicBase {
    protected:
        T value_;

    public:
        NonAtomicBase(T value) : value_(value) {}

        T operator=(T desired) noexcept {
            return value_ = desired;
        }

        T operator=(T desired) volatile noexcept {
            return value_ = desired;
        }

        bool is_lock_free() const noexcept {
            return true;
        }
        bool is_lock_free() const volatile noexcept {
            return true;
        }

        void store(T desired, std::memory_order = std::memory_order_seq_cst) noexcept {
            value_ = desired;
        }

        void store(T desired, std::memory_order = std::memory_order_seq_cst) volatile noexcept {
            value_ = desired;
        }

        T load( std::memory_order order = std::memory_order_seq_cst ) const noexcept {
            return value_;
	}

        T load( std::memory_order order = std::memory_order_seq_cst ) const volatile noexcept {
            return value_;
        }

        operator T() const noexcept {
            return value_;
        }
	
        operator T() const volatile noexcept {
            return value_;
        }

        T exchange( T desired, std::memory_order order = std::memory_order_seq_cst ) noexcept {
            return std::exchange(value_, desired);
        }

        T exchange( T desired, std::memory_order order = std::memory_order_seq_cst ) volatile noexcept {
            return std::exchange(value_, desired);
        }

    private:
        bool cas(T& expected, T desired) noexcept {
            assert(value_ == expected);
            value_ = desired;
            return true;
        }

    public:
        bool compare_exchange_weak(
            T& expected, T desired,
            std::memory_order = std::memory_order_seq_cst,
            std::memory_order = std::memory_order_seq_cst) noexcept
        {
            return cas(expected, desired);
        }

        bool compare_exchange_weak(
            T& expected, T desired,
            std::memory_order = std::memory_order_seq_cst,
            std::memory_order = std::memory_order_seq_cst) volatile noexcept
        {
            return cas(expected, desired);
        }

        bool compare_exchange_strong(
            T& expected, T desired,
            std::memory_order = std::memory_order_seq_cst,
            std::memory_order = std::memory_order_seq_cst) noexcept
        {
            return cas(expected, desired);
        }

        bool compare_exchange_strong(
            T& expected, T desired,
            std::memory_order = std::memory_order_seq_cst,
            std::memory_order = std::memory_order_seq_cst) volatile noexcept
        {
            return cas(expected, desired);
        }
    };

    template <typename T, class = void>
    class NonAtomic;

    // Template specialization for pointers
    template <typename T>
    class NonAtomic<T*> : public NonAtomicBase<T*> {
        using Base = NonAtomicBase<T*>;
    public:
        using Base::Base;
        NonAtomic(const NonAtomic&) = delete;
        NonAtomic& operator=(const NonAtomic&) = delete;
        NonAtomic& operator=(const NonAtomic&) volatile = delete;

        T* fetch_add( std::ptrdiff_t arg, std::memory_order order = std::memory_order_seq_cst ) noexcept {
            return Base::exchange(Base::value_ + arg);
        }
	
        T* fetch_add( std::ptrdiff_t arg, std::memory_order order = std::memory_order_seq_cst ) volatile noexcept {
            return Base::exchange(Base::value_ + arg);
        }

        T* operator++() noexcept {
            return ++Base::value_;
        }

        T* operator++() volatile noexcept {
            return ++Base::value_;
        }

        T* operator++(int) noexcept {
            return Base::value_++;
        }

        T* operator++(int) volatile noexcept {
            return Base::value_++;
        }
    };

    // Template specialization for integral types
    template <typename T>
    class NonAtomic<T, std::enable_if_t<std::is_integral_v<T>>> : public NonAtomicBase<T> {
        using Base = NonAtomicBase<T>;
    public:
        using Base::Base;
        NonAtomic(const NonAtomic&) = delete;
        NonAtomic& operator=(const NonAtomic&) = delete;
        NonAtomic& operator=(const NonAtomic&) volatile = delete;

        T fetch_add( T arg, std::memory_order order = std::memory_order_seq_cst ) noexcept {
            return Base::exchange(Base::value_ + arg);
        }
	
        T fetch_add( T arg, std::memory_order order = std::memory_order_seq_cst ) volatile noexcept {
            return Base::exchange(Base::value_ + arg);
        }

        T operator++() noexcept {
            return ++Base::value_;
        }

        T operator++() volatile noexcept {
            return ++Base::value_;
        }

        T operator++(int) noexcept {
            return Base::value_++;
        }

        T operator++(int) volatile noexcept {
            return Base::value_++;
        }
    };

    // Template specialization for floating point types
    template <typename T>
    class NonAtomic<T, std::enable_if_t<std::is_floating_point_v<T>>> : public NonAtomicBase<T> {
        using Base = NonAtomicBase<T>;
    public:
        using Base::Base;
        NonAtomic(const NonAtomic&) = delete;
        NonAtomic& operator=(const NonAtomic&) = delete;
        NonAtomic& operator=(const NonAtomic&) volatile = delete;
    };

    template <typename T, bool atomic>
    struct atomic_selector {
        using type = std::atomic<T>;
    };

    template <typename T>
    struct atomic_selector<T, false> {
        using type = NonAtomic<T>;
    };

    // Atom<T,atomic> is an optionally atomic type.  Atom<T,true> is
    // an alias for std::atomic<T>.  Atom<T>,false> is an alias for a
    // non-atomic implementation of std::atomic<T>--the object has the
    // same member methods, but does not do anything to ensure memory
    // orderings.  The compare_exchange_* methods on the non-atomic
    // version assert that the current value is the same as the
    // expected value--this should always be the case for when the
    // non-atomic types are used.
    template <typename T, bool atomic = true>
    using Atom = typename atomic_selector<T, atomic>::type;
}

#endif
