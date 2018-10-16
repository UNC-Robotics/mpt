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
#ifndef MPT_IMPL_PRRT_STAR_EDGE_DATA_HPP
#define MPT_IMPL_PRRT_STAR_EDGE_DATA_HPP

#include "node.hpp"
#include <atomic>
#include <cassert>
#include <variant>

namespace unc::robotics::mpt::impl::prrt_star {
    // EdgeData is used for storing the result of link() in the
    // concurrent version of PRRT*.  The main problem this solves it
    // removing the potentially expensive copy and memory requirements
    // need to for the multiple copies of edges that PRRT* generates.
    // 
    // The non-concurrent version of RRT* uses ScenarioLinkStore as it
    // does not make copies.
    template <typename T>
    class EdgeData {
        std::shared_ptr<T> link_;
    public:
        EdgeData(T&& link)
            : link_(std::make_shared<T>(std::move(link)))
        {
        }

        const T& link() const {
            return *link_;
        }
    };

    // When link() returns a bool, there's nothing to store.
    template <>
    class EdgeData<std::monostate> {
    public:
        EdgeData(std::monostate) {}

        std::monostate link() const {
            return {};
        }
    };

    // When link() returns a pointer, we have no additional reference
    // management requirements, and we can just copy the link from one
    // edge to its replacement.
    template <typename T>
    class EdgeData<T*> {
        T* link_;
    public:
        EdgeData(T* link) : link_(link) {}

        const T* link() const {
            return link_;
        }
    };

    // When link() returns a shared_ptr<T>, we also store a shared
    // ptr, and increase the reference count as we copy from one link
    // to its replacement.
    template <typename T>
    class EdgeData<std::shared_ptr<T>> {
        std::shared_ptr<T> link_;
    public:
        EdgeData(std::shared_ptr<T>&& link) : link_(std::move(link)) {}

        const std::shared_ptr<T>& link() const {
            return link_;
        }
    };

    // For the std::unique_ptr<T> case, we need to keep multiple
    // copies of it around.  We transfer the unique_ptr reference to
    // the shared_ptr<T> to enable copying with reference counting.
    // The 'uniqueness' of the trajectory is maintained within the
    // graph.
    template <typename T>
    class EdgeData<std::unique_ptr<T>> {
        std::shared_ptr<T> link_;
    public:
        EdgeData(std::unique_ptr<T>&& link) : link_(std::move(link)) {}

        const T* link() const {
            return link_->get();
        }
    };

    // TODO:
    // template <typename T>
    // class EdgeData<std::weak_ptr<T>, true> {
    // };
}

#endif
