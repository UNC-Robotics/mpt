#include <mpt/impl/thread_pool.hpp>
#include "test.hpp"
#include <set>

TEST(run_in_pool) {    
    using namespace unc::robotics::mpt::impl;
    using namespace std::literals;
    
    static constexpr int n = 32;
    ThreadPool pool(n);
    std::atomic_int count{0};

    for (int i=1 ; i<n ; ++i)
        pool.submit([&count] { ++count; while (count != n); });

    ++count;
    using Clock = std::chrono::steady_clock;
    auto start = Clock::now();
    while (count != n && Clock::now() - start < 1s)
        ;

    std::cout << "JOB QUEUE DONE!" << std::endl;
    
    EXPECT(count.load()) == n;
}

TEST(reuse_pool) {
    using namespace unc::robotics::mpt::impl;
    using namespace std::literals;
    static constexpr int poolSize = 4;
    ThreadPool pool(poolSize);
    std::mutex mutex;
    std::condition_variable waitToReturn;
    std::condition_variable allDone;
    std::set<std::thread::id> firstThreads;
    
    for (int iter=0 ; iter<2 ; ++iter) {
        std::atomic_int doneCount{0};
        std::set<std::thread::id> threads;
        
        for (int i=1 ; i<poolSize ; ++i)
            pool.submit(
                [&] {
                    bool notifyMain;
                    {
                        std::unique_lock<std::mutex> lock(mutex);
                        threads.emplace(std::this_thread::get_id());
                        waitToReturn.wait(lock, [&] { return threads.size() == poolSize-1; });
                        waitToReturn.notify_one();
                    }
                    // make sure that the main thread does not
                    // continue until we've released the lock and will
                    // no longer use the condition variables.
                    if (++doneCount == poolSize - 1)
                        allDone.notify_one();
                });

        std::unique_lock<std::mutex> lock(mutex);
        allDone.wait(lock, [&] { return doneCount == poolSize-1; });
        
        EXPECT(threads.size()) == poolSize-1;

        if (iter == 0) {
            firstThreads = threads;
        } else {
            for (auto id : threads)
                EXPECT(firstThreads.count(id)) == 1;
        }
    }        
}

TEST(use_env_variable) {
    using namespace unc::robotics::mpt::impl;
    
    const char *old = getenv("OMP_NUM_THREADS");
    std::string oldCopy;
    if (old)
        oldCopy = old;

    // check that if OMP_NUM_THREADS is not set, that it uses
    // hardware_concurrency by default
    unsetenv("OMP_NUM_THREADS");
    {
        ThreadPool pool;
        EXPECT(pool.size()) == std::max(1u, std::thread::hardware_concurrency());
    }

    // If pool size is specified, that takes precedence
    {
        ThreadPool pool(3);
        EXPECT(pool.size()) == 3;
    }

    // If OMP_NUM_THREADS is specified, use it
    setenv("OMP_NUM_THREADS", "5", 1);
    {
        ThreadPool pool;
        EXPECT(pool.size()) == 5;
    }

    // If pool size is specified, that takes precedence
    {
        ThreadPool pool(7);
        EXPECT(pool.size()) == 7;
    }

    // If the environment variable is invalid, then use hardware
    // concurrency.  (it will also display a message to the user)
    setenv("OMP_NUM_THREADS", "one", 1);
    {
        ThreadPool pool;
        EXPECT(pool.size()) == std::max(1u, std::thread::hardware_concurrency());
    }
    

    if (old) {
        setenv("OMP_NUM_THREADS", oldCopy.c_str(), 1);
    } else {
        unsetenv("OMP_NUM_THREADS");
    }
}

