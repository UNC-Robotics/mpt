#include <mpt/log.hpp>
#include <mpt/lp_space.hpp>
#include <mpt/box_bounds.hpp>
#include <mpt/goal_state.hpp>
#include <mpt/planner.hpp>
#include "test.hpp"
#include <fstream> // TODO: <-- remove
#include <optional>

namespace mpt_test {
    template <typename Pt, typename S0, typename S1>
    typename Pt::Scalar distPointSegmentSquared(
        const Eigen::MatrixBase<Pt>& pt,
        const Eigen::MatrixBase<S0>& s0,
        const Eigen::MatrixBase<S1>& s1)
    {
        using Scalar = typename Pt::Scalar;
        using Vector = typename Pt::PlainMatrix;
            
        Vector v = s1 - s0;
        Vector w = pt - s0;
        Scalar c1 = v.dot(w);
        if (c1 <= 0)
            return w.squaredNorm();
        Scalar c2 = v.squaredNorm();
        if (c2 <= c1)
            return (pt - s1).squaredNorm();
        return (s0 - pt + v * (c1 / c2)).squaredNorm();
    }
    
    template <typename Scalar = double, int dimensions = 3>
    class TestScenarioBase {
        static_assert(dimensions > 1, "must be 2D or greater");
        
    public:
        using Space = unc::robotics::mpt::L2Space<Scalar, dimensions>;
        using Bounds = unc::robotics::mpt::BoxBounds<Scalar, dimensions>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Goal = unc::robotics::mpt::GoalState<Space>;

        static Scalar defaultObstacleRadius() {
            return std::sqrt((Scalar)(dimensions - 1)) * 0.95;
        }

        static State goalState() {
            Scalar x = (std::sqrt((Scalar)dimensions) - defaultObstacleRadius()) / 2;
            assert(0 < x && x < 1);
            
            State q;
            q.fill((Scalar)1 - x);
            return q;
        }

        static State startState() {
            return -goalState();
        }

    private:
        static Bounds makeBounds() {
            Eigen::Matrix<Scalar, dimensions, 1> min, max;
            min.fill(-1);
            max.fill( 1);
            return Bounds(min, max);
        }

        Space space_;
        Bounds bounds_{makeBounds()};
        Goal goal_{1e-6, goalState()};
        
    public:
        const Space& space() const {
            return space_;
        }

        const Bounds& bounds() const {
            return bounds_;
        }

        const Goal& goal() const {
            return goal_;
        }
    };

    template <typename Scalar = double, int dimensions = 3>
    class BasicScenario : public TestScenarioBase<Scalar, dimensions> {
        using Base = TestScenarioBase<Scalar, dimensions>;

        Scalar centerObstacleRadiusSquared_;

    public:
        using typename Base::State;
        
        explicit BasicScenario(Scalar r = Base::defaultObstacleRadius())
            : centerObstacleRadiusSquared_(r*r)
        {
            MPT_LOG(DEBUG) << "radius=" << r;
        }
        
        bool valid(const State& q) const {
            return q.squaredNorm() > centerObstacleRadiusSquared_;
        }

        bool link(const State& a, const State& b) const {
            return distPointSegmentSquared(State::Zero(), a, b) > centerObstacleRadiusSquared_;
        }
    };

    template <typename Scalar = double, int dimensions = 3>
    class TrajectoryScenario : public TestScenarioBase<Scalar, dimensions> {
        using Base = TestScenarioBase<Scalar, dimensions>;

        Scalar centerObstacleRadiusSquared_;
    public:

        using typename Base::State;
        
        explicit TrajectoryScenario(Scalar r = Base::defaultObstacleRadius())
            : centerObstacleRadiusSquared_(r*r)
        {
            MPT_LOG(DEBUG) << "radius=" << r;
        }
        
        bool valid(const State& q) const {
            return q.squaredNorm() > centerObstacleRadiusSquared_;
        }

        std::optional<std::vector<State>> link(const State& a, const State& b) const {
            if (distPointSegmentSquared(State::Zero(), a, b) <= centerObstacleRadiusSquared_)
                return {};

            unsigned steps = std::ceil((a - b).norm() * 10);
            std::vector<State> traj;
            traj.resize(steps+1);
            traj[0] = a;
            for (unsigned i=1 ; i<steps ; ++i)
                traj[i] = a + (b - a) * (Scalar(i) / steps);
            traj[steps] = b;
            return traj;
        }        
    };

    template <typename Scalar = double, int dimensions = 3>
    class SharedTrajectoryScenario : public TestScenarioBase<Scalar, dimensions> {
        using Base = TestScenarioBase<Scalar, dimensions>;

        Scalar centerObstacleRadiusSquared_;
    public:

        using typename Base::State;
        
        explicit SharedTrajectoryScenario(Scalar r = Base::defaultObstacleRadius())
            : centerObstacleRadiusSquared_(r*r)
        {
            MPT_LOG(DEBUG) << "radius=" << r;
        }
        
        bool valid(const State& q) const {
            return q.squaredNorm() > centerObstacleRadiusSquared_;
        }

        std::shared_ptr<std::tuple<State, State>> link(const State& a, const State& b) const {
            if (distPointSegmentSquared(State::Zero(), a, b) <= centerObstacleRadiusSquared_)
                return {};

            return std::make_shared<std::tuple<State, State>>(a, b);
        }        
    };

    template <typename T, typename Q, typename = void>
    struct has_add_goal : std::false_type {};
    template <typename T, typename Q>
    struct has_add_goal<T, Q, std::void_t<decltype(std::declval<T>().addGoal(std::declval<Q>()))>>
        : std::true_type {};

    template <typename Algorithm>
    void testSolvingBasicScenario() {
        using Scalar = double;
        static constexpr int dim = 3;
        using namespace unc::robotics;
        using namespace mpt;
        using namespace mpt_test;
        using namespace std::literals;
        using Scenario = BasicScenario<Scalar, dim>;
        // 10 seconds should be more than enough time to solve any of these.
        static constexpr auto MAX_SOLVE_TIME = 10s;
    
        Planner<Scenario, Algorithm> planner;
        
        planner.addStart(Scenario::startState());
        if constexpr (has_add_goal<Planner<Scenario, Algorithm>, decltype(Scenario::goalState())>::value)
            planner.addGoal(Scenario::goalState());

        planner.solveFor([&] { return planner.solved(); }, MAX_SOLVE_TIME);
        planner.printStats();

        using State = Eigen::Matrix<Scalar, dim, 1>;

        EXPECT(planner.solved()) == true;

        std::vector<State> solution = planner.solution();
        EXPECT(solution.size()) > 2; // start + end + at least one waypoint around obstacle
        EXPECT(solution[0] == Scenario::startState()) == true;
        EXPECT(solution.back() == Scenario::goalState()) == true;

        auto sit = solution.begin();
        planner.solution([&] (const State& a) {
            EXPECT(*sit++ == a) == true;
        });
        EXPECT(sit == solution.end()) == true;
        
        // the following block outputs an svg file with the graph from
        // the plan.
#if 0
        std::ofstream out("test_output.svg");
        
        struct Visitor {
            std::ofstream& out_;
            State from;

            Visitor(std::ofstream& out) : out_(out) {}
            
            void vertex(const State& q) {
                from = q;
            }
            
            void edge(const State& to) {
                out_ << "<line "
                    "x1='" << from[0] << "' "
                    "y1='" << from[1] << "' "
                    "x2='" << to[0] << "' "
                    "y2='" << to[1] << "' />\n";
            }
        };

        out <<
            "<svg version='1.1' width='600' height='600' xmlns='http://www.w3.org/2000/svg'\n"
            "  viewBox='-1 -1 2 2'>\n"
            "<g style='stroke:black; stroke-width:0.001;'>\n";
       
        planner.visitGraph(Visitor(out));
        out <<
            "</g>\n"
            "</svg>";
#endif
        
    }

    template <typename Algorithm, typename Scalar = double, int dim = 3>
    void testSolvingTrajectoryScenario() {
        using namespace unc::robotics;
        using namespace mpt;
        using namespace mpt_test;
        using namespace std::literals;
        using Scenario = TrajectoryScenario<Scalar, dim>;
        using State = typename Scenario::State;
        using Trajectory = std::vector<State>;
        
        // 10 seconds should be more than enough time to solve any of these.
        static constexpr auto MAX_SOLVE_TIME = 10s;
    
        Planner<Scenario, Algorithm> planner;
        
        planner.addStart(Scenario::startState());
        if constexpr (has_add_goal<Planner<Scenario, Algorithm>, decltype(Scenario::goalState())>::value)
            planner.addGoal(Scenario::goalState());

        planner.solveFor([&] { return planner.solved(); }, MAX_SOLVE_TIME);
        planner.printStats();

        EXPECT(planner.solved()) == true;

        std::vector<State> solution = planner.solution();
        EXPECT(solution.size()) > 2; // start + end + at least one waypoint around the obstacle
        EXPECT(solution[0] == Scenario::startState()) == true;
        EXPECT(solution.back() == Scenario::goalState()) == true;
        auto sit = solution.begin();
        
        State prev = Scenario::startState();
        planner.solution(
            [&] (const State& a, const Trajectory& traj, const State& b, bool forward) {
                EXPECT(prev == a) == true;
                EXPECT(*sit == a) == true;
                EXPECT(traj.size()) >= 2;
                if (forward) {
                    EXPECT(traj[0] == a) == true;
                    EXPECT(traj.back() == b) == true;
                } else {
                    EXPECT(traj.back() == a) == true;
                    EXPECT(traj[0] == b) == true;
                }
                EXPECT(*++sit == b) == true;
                prev = b;
            });
        
        EXPECT(prev == Scenario::goalState()) == true;
    }

    template <typename Algorithm, typename Scalar = double, int dim = 3>
    void testSolvingSharedTrajectoryScenario() {
        using namespace unc::robotics;
        using namespace mpt;
        using namespace mpt_test;
        using namespace std::literals;
        using Scenario = SharedTrajectoryScenario<Scalar, dim>;
        using State = typename Scenario::State;
        using Trajectory = std::vector<State>;
        
        // 10 seconds should be more than enough time to solve any of these.
        static constexpr auto MAX_SOLVE_TIME = 10s;
    
        Planner<Scenario, Algorithm> planner;
        
        planner.addStart(Scenario::startState());
        if constexpr (has_add_goal<Planner<Scenario, Algorithm>, decltype(Scenario::goalState())>::value)
            planner.addGoal(Scenario::goalState());

        planner.solveFor([&] { return planner.solved(); }, MAX_SOLVE_TIME);
        planner.printStats();

        EXPECT(planner.solved()) == true;

        State prev = Scenario::startState();
        planner.solution(
            [&] (const State& a, const std::shared_ptr<std::tuple<State, State>>& traj, const State& b, bool forward) {
                EXPECT(prev == a) == true;
                if (forward) {
                    EXPECT(std::get<0>(*traj) == a) == true;
                    EXPECT(std::get<1>(*traj) == b) == true;
                } else {
                    EXPECT(std::get<1>(*traj) == a) == true;
                    EXPECT(std::get<0>(*traj) == b) == true;
                }
                prev = b;
            });
        
        EXPECT(prev == Scenario::goalState()) == true;
        
        prev = Scenario::startState();
        planner.solution(
            [&] (const State& a, const std::tuple<State, State>& traj, const State& b, bool forward) {
                EXPECT(prev == a) == true;
                if (forward) {
                    EXPECT(std::get<0>(traj) == a) == true;
                    EXPECT(std::get<1>(traj) == b) == true;
                } else {
                    EXPECT(std::get<1>(traj) == a) == true;
                    EXPECT(std::get<0>(traj) == b) == true;
                }
                prev = b;
            });
        
        EXPECT(prev == Scenario::goalState()) == true;

    }

}

