#define MPT_LOG_LEVEL WARN
#include "planner_integration_test.hpp"
#include <mpt/pprm.hpp>
#include <nigh/gnat.hpp>
#include <nigh/linear.hpp>

using namespace unc::robotics;
using namespace mpt;
using namespace mpt_test;

TEST(pprm_until_solved) {
    testSolvingBasicScenario<PPRM<>>();
}

TEST(pprm_until_solved_with_stats) {
    testSolvingBasicScenario<PPRM<report_stats<true>>>();
}

TEST(pprm_until_solved_single_threaded) {
    testSolvingBasicScenario<PPRM<single_threaded>>();
}

TEST(pprm_until_solved_with_gnat) {
    testSolvingBasicScenario<PPRM<nigh::GNAT<>>>();
}

TEST(pprm_until_solved_with_linear) {
    testSolvingBasicScenario<PPRM<nigh::Linear>>();
}

TEST(pprm_options_parser) {
    // The options parser should resolve the following two planners to
    // the same concrete type.
    using Scenario = BasicScenario<>;
    using A = mpt::Planner<Scenario, PPRM<nigh::Linear, single_threaded, report_stats<true>>>;
    using B = mpt::Planner<Scenario, PPRM<single_threaded, report_stats<true>, nigh::Linear>>;
    EXPECT((std::is_same_v<A, B>)) == true;
}

TEST(pprm_with_trajectory) {
    testSolvingTrajectoryScenario<PPRM<>>();
}

TEST(pprm_with_shared_trajectory) {
    testSolvingSharedTrajectoryScenario<PPRM<>>();
}

