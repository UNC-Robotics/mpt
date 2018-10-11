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
#ifndef MPT_IMPL_OBJECT_POOL_HPP
#define MPT_IMPL_OBJECT_POOL_HPP

#include <deque>
#include <forward_list>

namespace unc::robotics::mpt::impl {
    // An ObjectPool is an (optionally) block-allocated, moveable,
    // collection of objects.  The primary purpose of these
    // collections is to make cleanup of objects easier.  ObjectPools
    // retain ownership of the objects they allocate, and thus when
    // they are destroyed, so are the objects allocated from them.
    // Thus the typical usage is to have ObjectPools follow the
    // lifespan of an object that creates a lot of little objects
    // (e.g. the graph of a planner).
    //
    // Once an object is allocated from the pool it will remain valid,
    // until the pool is destroyed.  Thus the only exposed methods of
    // a pool are guaranteed to keep pointers valid.  It also means
    // that ObjectPools are movable, but not copiable, as copying
    // would have ill-defined semantics when it comes to the resulting
    // pointers.
    //
    // ObjectPools that are block-allocated can also provide a
    // performance boost over individually allocated objects.
    template <typename T, bool block = true, class Allocator = std::allocator<T>>
    class ObjectPool;

    // The block-allocated specialization of ObjectPool currently uses
    // a std::deque as its underlying container implementation.  A
    // std::deque performs block allocation, and does not invalidate
    // pointers (like std::vector would).
    template <typename T, class Allocator>
    class ObjectPool<T, true, Allocator> : std::deque<T, Allocator> {
        using Base = std::deque<T, Allocator>;
    public:
        // Delete the copy constructor--it does not typically make
        // sense under intended usage since the resulting copy will
        // have invalid pointers.
        ObjectPool(const ObjectPool&) = delete;

        ObjectPool() {
        }

        ObjectPool(ObjectPool&& other)
            : Base(std::move(other))
        {
        }

        template <typename ... Args>
        T* allocate(Args&& ... args) {
            Base::emplace_back(std::forward<Args>(args)...);
            return &Base::back();
        }

        using Base::begin;
        using Base::end;
    };

    // The non-block-allocated specialization for ObjectPool currently
    // uses a std::forward_list as its underlying implementation.
    template <typename T, class Allocator>
    class ObjectPool<T, false, Allocator> : std::forward_list<T, Allocator> {
        using Base = std::forward_list<T, Allocator>;
    public:
        ObjectPool(const ObjectPool&) = delete;

        ObjectPool() {
        }

        ObjectPool(ObjectPool&& other)
            : Base(std::move(other))
        {
        }

        template <typename ... Args>
        T* allocate(Args&& ... args) {
            Base::emplace_front(std::forward<Args>(args)...);
            return &Base::front();
        }

        using Base::begin;
        using Base::end;
    };
}

#endif
