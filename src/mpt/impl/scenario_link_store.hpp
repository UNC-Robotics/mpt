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
#ifndef MPT_IMPL_SCENARIO_LINK_STORE_HPP
#define MPT_IMPL_SCENARIO_LINK_STORE_HPP

#include "scenario_state.hpp"
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

namespace unc::robotics::mpt::impl {
    template <class T>
    class ScenarioLinkStore;

    template <>
    class ScenarioLinkStore<bool> {
    public:
        ScenarioLinkStore(bool) {}
    };

    template <typename T>
    class ScenarioLinkStoreValue {
        T link_;
        
    public:
        ScenarioLinkStoreValue(T&& link)
            : link_(std::move(link))
        {
        }

        const T& link() const {
            return link_;
        }
    };

    template <typename T>
    class ScenarioLinkStore<std::optional<T>> : public ScenarioLinkStoreValue<T> {
    public:
        ScenarioLinkStore(std::optional<T>&& link)
            : ScenarioLinkStoreValue<T>(std::move(link.value()))
        {
        }
    };

    template <typename T, typename State>
    class ScenarioLinkStore<std::pair<T, State>> : public ScenarioLinkStoreValue<T> {
    public:
        ScenarioLinkStore(std::pair<T, State>&& link)
            : ScenarioLinkStoreValue<T>(std::get<0>(std::move(link)))
        {
        }
    };

    template <typename T, typename State>
    class ScenarioLinkStore<std::tuple<T, State>> : public ScenarioLinkStoreValue<T> {
    public:
        ScenarioLinkStore(std::tuple<T, State>&& link)
            : ScenarioLinkStoreValue<T>(std::get<0>(std::move(link)))
        {
        }
    };

    template <typename Ptr>
    class ScenarioLinkStorePointer {
        Ptr link_;
    public:
        ScenarioLinkStorePointer(Ptr&& link)
            : link_(std::move(link))
        {
        }

        auto& link() const {
            return *link_;
        }
    };

    template <typename T>
    class ScenarioLinkStore<T*> : public ScenarioLinkStorePointer<T*> {
    public:
        ScenarioLinkStore(T* ptr)
            : ScenarioLinkStorePointer<T*>(ptr)
        {
        }
    };

    template <typename T>
    class ScenarioLinkStore<std::shared_ptr<T>>
        : public ScenarioLinkStorePointer<std::shared_ptr<T>>
    {
    public:
        ScenarioLinkStore(std::shared_ptr<T>&& ptr)
            : ScenarioLinkStorePointer<std::shared_ptr<T>>(std::move(ptr))
        {
        }
    };

    template <typename T, typename D>
    class ScenarioLinkStore<std::unique_ptr<T, D>>
        : public ScenarioLinkStorePointer<std::unique_ptr<T, D>>
    {
    public:
        ScenarioLinkStore(std::unique_ptr<T, D>&& ptr)
            : ScenarioLinkStorePointer<std::unique_ptr<T, D>>(std::move(ptr))
        {
        }
    };

    template <typename T>
    class ScenarioLinkStore<std::weak_ptr<T>> {
        std::weak_ptr<T> link_;
    public:
        ScenarioLinkStore(std::weak_ptr<T>&& ptr)
            : link_(std::move(ptr))
        {
        }

        const T& link() const {
            // THIS IS DANGEROUS and inefficient.  In order to access
            // value in a weak pointer we must lock it with a shared
            // pointer.  Normally the shared_ptr should remain in
            // scope during the usage of the pointer to prevent the
            // value from being deleted during usage.  Here we are
            // assuming that the shared trajectory object will remain
            // valid during the caller's usage of it.  On the
            // efficiency side, we are incrementing and decrementing a
            // reference count for no reason here (other than the API
            // requires it).  However the correct fix is not on the
            // efficiency side, but on the correctness side.
            std::shared_ptr<T> lock(link_);
            return *lock;
        }            
    };
}

#endif
