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
#ifndef MPT_IMPL_THREAD_POOL_HPP
#define MPT_IMPL_THREAD_POOL_HPP

#include "../log.hpp"
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <utility>
#include <cstdlib>

namespace unc::robotics::mpt::impl {
    
    enum ThreadPoolState {
        kThreadPoolRunning,
        kThreadPoolShuttingDown,
        kThreadPoolShutdown
    };

    class JobQueue {
        std::mutex mutex_;
        std::condition_variable ready_;
        std::queue<std::function<void()>> jobs_;

        template <typename Fn>
        void queue(Fn&& fn) {
            std::unique_lock<std::mutex> lock(mutex_);
            jobs_.emplace(std::forward<Fn>(fn));
            lock.unlock();
        }
        
    public:
        template <typename Fn>
        void submit(Fn&& fn) {
            queue(std::forward<Fn>(fn));
            ready_.notify_one();
        }

        void shutdown() {
            queue(std::function<void()>{});
            ready_.notify_all(); 
        }

        std::function<void()> pop() {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [&] { return !jobs_.empty(); });
            if (!jobs_.front())
                return {};
            
            std::function<void()> fn{std::move(jobs_.front())};
            jobs_.pop();
            return fn;
        }
    };

    class ThreadPool {
        std::shared_ptr<JobQueue> jobQueue_{std::make_shared<JobQueue>()};
        std::vector<std::thread> threads_;

        static void threadMain(unsigned no, std::shared_ptr<JobQueue> jobQueue) {
            // MPT_LOG(DEBUG) << "thread " << no << " started";
            
            for (std::function<void()> fn ; !!(fn = jobQueue->pop()) ; ) {
                try {
                    fn();
                } catch (const std::exception& ex) {
                    MPT_LOG(ERROR) << "event terminated with exception: " << ex.what();
                }
            }
            
            // MPT_LOG(DEBUG) << "thread " << no << " terminated";
        }
    public:
        // not movable or copyable
        ThreadPool(ThreadPool&&) = delete;
        ThreadPool(const ThreadPool&) = delete;
        
        // 0 means "use default" (i.e., OMP_NUM_THREADS or hardware concurrency)
        ThreadPool(unsigned nThreads = 0) {
            unsigned hw = std::thread::hardware_concurrency();
            if (nThreads == 0) {
                nThreads = hw; 
                if (const char *envThreadStr = getenv("OMP_NUM_THREADS")) {
                    char *endp;
                    long envThreads = std::strtol(envThreadStr, &endp, 0);
                    if (envThreads <= 0 || *endp || envThreads == std::numeric_limits<long>::max()) {
                        MPT_LOG(WARN) << "invalid value for OMP_NUM_THREADS, ignoring";
                    } else {
                        nThreads = envThreads;
                    }
                }
            }
            
            if (nThreads < 1) {
                nThreads = 1;
            } else if (nThreads > hw) {
                MPT_LOG(WARN) << "specified concurrency (" << nThreads
                              << ") exceeds hardware concurrency (" << hw << ')';
            }

            // Start with 1, since we use the caller's thread as thread 0.
            for (unsigned no=1 ; no < nThreads ; ++no)
                threads_.emplace_back(&ThreadPool::threadMain, no, jobQueue_);
        }

        ~ThreadPool() {
            jobQueue_->shutdown();
            for (auto& t : threads_)
                t.join();
        }

        static ThreadPool& singleton() {
            static ThreadPool threadPool;
            return threadPool;
        }

        std::size_t size() const {
            return threads_.size() + 1;
        }

        template <typename Fn>
        void submit(Fn&& fn) {
            jobQueue_->submit(std::forward<Fn>(fn));
        }
    };
}

#endif
