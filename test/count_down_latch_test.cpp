#include <mpt/impl/count_down_latch.hpp>
#include "test.hpp"
#include <thread>
#include <list>

TEST(count_down) {
    using namespace unc::robotics::mpt::impl;
    int n = 111;
    CountDownLatch latch(n);
    std::atomic_bool started{false};
    std::atomic_bool finished{false};
    
    std::thread waiter([&] { started = true; latch.wait(); finished = true; });

    while (!started.load())
        ;
    
    std::list<std::thread> threads;
    for (int i=0 ; i<n ; ++i) {
        // the waiting thread should not be finished until all other
        // threads have invoked countDown()
        EXPECT(finished.load()) == false;
        threads.emplace_back([&] { latch.countDown(); });
        std::this_thread::yield();
    }

    for (auto& t : threads)
        t.join();

    using namespace std::literals;
    using Clock = std::chrono::steady_clock;
    auto start = Clock::now();
    // the waiting thread should now be finished.  We allow for at
    // most 5 seconds before we fail th test (which should be
    // rediculously long, except in bizarre cases))
    while ((Clock::now() - start) < 5s && finished.load() == false)
        ;
    
    EXPECT(finished.load()) == true;
    waiter.join();
}
