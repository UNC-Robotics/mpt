#define MPT_LOG_LEVEL WARN
#include "planner_integration_test.hpp"
#include <mpt/prrt.hpp>
#include <nigh/gnat.hpp>
#include <nigh/linear.hpp>

using namespace unc::robotics;
using namespace mpt;
using namespace mpt_test;

TEST(prrt_until_solved) {
    testSolvingBasicScenario<PRRT<>>();
}

TEST(prrt_until_solved_with_stats) {
    testSolvingBasicScenario<PRRT<report_stats<true>>>();
}

TEST(prrt_until_solved_single_threaded) {
    testSolvingBasicScenario<PRRT<single_threaded>>();
}

TEST(prrt_until_solved_with_gnat) {
    testSolvingBasicScenario<PRRT<nigh::GNAT<>>>();
}

TEST(prrt_until_solved_with_linear) {
    testSolvingBasicScenario<PRRT<nigh::Linear>>();
}

TEST(prrt_options_parser) {
    // The options parser should resolve the following two planners to
    // the same concrete type.
    using Scenario = BasicScenario<>;
    using A = mpt::Planner<Scenario, PRRT<nigh::Linear, single_threaded, report_stats<true>>>;
    using B = mpt::Planner<Scenario, PRRT<single_threaded, report_stats<true>, nigh::Linear>>;
    EXPECT((std::is_same_v<A, B>)) == true;
}
