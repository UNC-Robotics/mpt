#define MPT_LOG_LEVEL WARN
#include "planner_integration_test.hpp"
#include <mpt/prrt.hpp>
#include <nigh/gnat.hpp>
#include <nigh/linear.hpp>

using namespace unc::robotics::mpt;
using namespace mpt_test;
namespace nigh = unc::robotics::nigh;

TEST(prrt_until_solved_with_goal_class) {
    testSolvingBasicScenario<PRRT<>, TEST_GOAL_KIND_CLASS>();
}

TEST(prrt_until_solved_with_goal_method) {
    testSolvingBasicScenario<PRRT<>, TEST_GOAL_KIND_METHOD>();
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
    using A = Planner<Scenario, PRRT<nigh::Linear, single_threaded, report_stats<true>>>;
    using B = Planner<Scenario, PRRT<single_threaded, report_stats<true>, nigh::Linear>>;
    EXPECT((std::is_same_v<A, B>)) == true;
}

TEST(prrt_with_trajectory) {
    testSolvingTrajectoryScenario<PRRT<>>();
}

TEST(prrt_with_shared_trajectory) {
    testSolvingSharedTrajectoryScenario<PRRT<>>();
}
