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
#ifndef MPT_IMPL_WORKER_POOL_OPENMP_HPP
#define MPT_IMPL_WORKER_POOL_OPENMP_HPP

#include "../log.hpp"
#include "finally.hpp"
#include <omp.h>
#include <vector>
#include <stdexcept>

namespace unc::robotics::mpt::impl {

    template <typename T, int maxThreads, typename Allocator>
    class WorkerPool {
        static_assert(maxThreads >= 0, "maxThreads must be non-negative");

        std::vector<T, Allocator> workers_;
        std::atomic_bool solving_{false};

    public:
        using value_type = T;
        using iterator = typename std::vector<T, Allocator>::iterator;
        using cosnt_iterator = typename std::vector<T, Allocator>::const_iterator;
        
        WorkerPool(WorkerPool&& other)
            : workers_(std::move(other.workers_))
        {
            assert(!other.solving_);
        }

        // this is not Args&& since we need to create multiple copies,
        // using std::forward<Args>(args)... could leave an argument
        // in a bad state.
        template <typename ... Args>
        WorkerPool(const Args& ... args) {
            // Note: omp_get_num_procs() is the hardware concurrency
            // level.  omp_get_max_threads() resepects the
            // OMP_NUM_THREADS=n environment variable.  (n == 1 sets
            // single-threaded)

            unsigned nThreads = std::max(1, omp_get_max_threads());
            if (maxThreads != 0 && nThreads > (unsigned)maxThreads) {
                MPT_LOG(TRACE) << "limiting threads to " << maxThreads
                               << ", which is less than hardware concurrency ("
                               << nThreads << ")";
                nThreads = maxThreads;
            }

            workers_.reserve(nThreads);
            for (unsigned no=0 ; no<nThreads ; ++no)
                workers_.emplace_back(no, args...);
        }

        unsigned size() const {
            return static_cast<unsigned>(workers_.size());
        }

        T& operator[] (std::size_t i) {
            return workers_[i];
        }

        const T& operator[] (std::size_t i) const {
            return workers_[i];
        }

        template <typename Context, typename DoneFn>
        void solve(Context& context, const DoneFn& doneFn) {
            // TODO: this should really be an assert, it indicates a
            // program logic error, not a runtime error.
            if (solving_.exchange(true))
                throw std::runtime_error("already solving");
            auto unsolving = finally([&]() { solving_ = false; });

            unsigned nThreads = size();
            MPT_LOG(INFO) << "solving with " << nThreads << " threads";
            if (nThreads == 1) {
                workers_[0].solve(context, doneFn);
            } else {
                std::atomic_bool done{false};
#pragma omp parallel for shared(done) schedule(static, 1) num_threads(nThreads)
                for (unsigned i = 0 ; i<nThreads ; ++i) {
                    try {
                        if (int tNo = omp_get_thread_num()) {
                            workers_[tNo].solve(context, [&] { return done.load(std::memory_order_relaxed); });
                        } else {
                            workers_[0].solve(context, doneFn);
                            done.store(true, std::memory_order_relaxed);
                        }
                    } catch (const std::exception& ex) {
                        MPT_LOG(ERROR) << "solve died with exception: " << ex.what();
                    }
                }
            }
        }

        // T& operator() {
        //     return workers_[omp_get_thread_num()];
        // }

        // const T& operator() const {
        //     return workers_[omp_get_thread_num()];
        // }

        auto begin() {
            return workers_.begin();
        }
        
        auto end() {
            return workers_.end();
        }
        
        auto begin() const {
            return workers_.begin();
        }
        
        auto end() const {
            return workers_.end();
        }
    };
}

#endif
