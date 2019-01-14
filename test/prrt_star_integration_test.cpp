#define MPT_LOG_LEVEL WARN
#include "planner_integration_test.hpp"
#include <mpt/prrt_star.hpp>
#include <nigh/gnat.hpp>
#include <nigh/linear.hpp>

using namespace unc::robotics;
using namespace mpt;
using namespace mpt_test;

TEST(prrt_star_until_solved) {
    testSolvingBasicScenario<PRRTStar<>>();
}

TEST(prrt_star_until_solved_with_stats) {
    testSolvingBasicScenario<PRRTStar<report_stats<true>>>();
}

TEST(prrt_star_until_solved_single_threaded) {
    testSolvingBasicScenario<PRRTStar<single_threaded>>();
}

TEST(prrt_star_until_solved_with_gnat) {
    testSolvingBasicScenario<PRRTStar<nigh::GNAT<>>>();
}

TEST(prrt_star_until_solved_with_linear) {
    testSolvingBasicScenario<PRRTStar<nigh::Linear>>();
}

TEST(prrt_start_until_solved_r_nearest) {
    testSolvingBasicScenario<PRRTStar<rewire_r_nearest>>();
}

TEST(prrt_start_until_solved_k_nearest) {
    testSolvingBasicScenario<PRRTStar<rewire_k_nearest>>();
}

TEST(prrt_star_options_parser) {
    // The options parser should resolve the following two planners to
    // the same concrete type.
    using Scenario = BasicScenario<>;
    using A = mpt::Planner<Scenario, PRRTStar<nigh::Linear, rewire_r_nearest, single_threaded, report_stats<true>>>;
    using B = mpt::Planner<Scenario, PRRTStar<single_threaded, report_stats<true>, nigh::Linear, rewire_r_nearest>>;
    EXPECT((std::is_same_v<A, B>)) == true;
}

TEST(prrt_star_options_default) {
    using Scenario = BasicScenario<>;
    using A = mpt::Planner<Scenario, PRRTStar<rewire_k_nearest, report_stats<false>, max_threads<0>>>;
    using B = mpt::Planner<Scenario, PRRTStar<>>;
    EXPECT((std::is_same_v<A, B>)) == true;
}

TEST(prrt_star_with_trajectory) {
    testSolvingTrajectoryScenario<PRRTStar<>>();
}

TEST(prrt_star_with_shared_trajectory) {
    testSolvingSharedTrajectoryScenario<PRRTStar<>>();
}
