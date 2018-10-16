#define MPT_LOG_LEVEL WARN
#include "planner_integration_test.hpp"
#include <mpt/pprm_irs.hpp>
#include <nigh/gnat.hpp>
#include <nigh/linear.hpp>

using namespace unc::robotics;
using namespace mpt;
using namespace mpt_test;

TEST(pprm_irs_until_solved) {
    testSolvingBasicScenario<PPRMIRS<>>();
}

TEST(pprm_irs_until_solved_with_stats) {
    testSolvingBasicScenario<PPRMIRS<report_stats<true>>>();
}

TEST(pprm_irs_until_solved_single_threaded) {
    testSolvingBasicScenario<PPRMIRS<single_threaded>>();
}

TEST(pprm_irs_until_solved_with_gnat) {
    testSolvingBasicScenario<PPRMIRS<nigh::GNAT<>>>();
}

TEST(pprm_irs_until_solved_with_linear) {
    testSolvingBasicScenario<PPRMIRS<nigh::Linear>>();
}

TEST(pprm_irs_options_parser) {
    // The options parser should resolve the following two planners to
    // the same concrete type.
    using Scenario = BasicScenario<>;
    using A = mpt::Planner<Scenario, PPRMIRS<nigh::Linear, single_threaded, report_stats<true>>>;
    using B = mpt::Planner<Scenario, PPRMIRS<single_threaded, report_stats<true>, nigh::Linear>>;
    EXPECT((std::is_same_v<A, B>)) == true;
}

TEST(pprm_irs_with_trajectory) {
    testSolvingTrajectoryScenario<PPRMIRS<>>();
}
#if 0
TEST(pprm_irs_with_shared_trajectory) {
    testSolvingSharedTrajectoryScenario<PPRMIRS<>>();
}
#endif
