#include <mpt/impl/worker_pool.hpp>
#include "test.hpp"

namespace mpt_test {
    class TestWorker {
        unsigned no_;
        
    public:
        TestWorker(unsigned no, double three) : no_(no) {
            EXPECT(three) == 3.0;
        }

        template <typename DoneFn>
        void solve(std::atomic_int& startCount, DoneFn done) {
            using namespace std::literals;
            --startCount;
            while (!done()) {
                std::this_thread::sleep_for(1ms);
            }
        }
    };
}

TEST(solve) {
    using namespace unc::robotics::mpt::impl;
    using namespace mpt_test;
    using namespace std::literals;
    
    WorkerPool<TestWorker> pool(3.0);

    EXPECT(pool.size()) > 0;

    using Clock = std::chrono::steady_clock;
    // normally the planner is passed in as the context.  Here we just
    // send in the atomic int to facilitate the test.
    std::atomic_int count{(int)pool.size()};
    pool.solve(count, [&, start = Clock::now()] {
        return count == 0 || Clock::now() - start > 60s;
    });

    EXPECT(count.load()) == 0;
}
