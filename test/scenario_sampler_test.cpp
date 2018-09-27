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

#include <mpt/impl/scenario_sampler.hpp>
#include <mpt/so3_space.hpp>
#include <mpt/lp_space.hpp>
#include <mpt/se3_space.hpp>
#include <mpt/box_bounds.hpp>
#include <mpt/cartesian_bounds.hpp>
#include <random>
#include <limits>
#include "test.hpp"

namespace mpt_test {
    using namespace unc::robotics::mpt;

    struct CustomSampler {
        using Space = SO3Space<double>;
        using State = typename Space::Type;

        template <typename RNG>
        State operator() (RNG& rng) const {
            return State(Eigen::Vector4d(1,-2,3,4).normalized());
        }
    };

    template <typename UInt>
    struct NotRandom {
        using result_type = UInt;

        static constexpr result_type min() {
            return std::numeric_limits<UInt>::min();
        }

        static constexpr result_type max() {
            return std::numeric_limits<UInt>::max();
        }

        result_type operator() () {
            return min();
        }
    };

    struct ScenarioWithSampleMethod {
        using Space = SO3Space<double>;
        using State = typename Space::Type;

        mutable int sampleCount_{0};

        template <typename RNG>
        State sample(RNG& rng) const {
            ++sampleCount_;

            return State(Eigen::Vector4d(-1,2,3,4).normalized());
        }

        // Note: even though this method exists, the sample method
        // should be used.
        CustomSampler sampler() const {
            return CustomSampler();
        }
    };

    struct ScenarioWithSamplerMethod {
        using Space = SO3Space<double>;
        using State = typename Space::Type;

        CustomSampler sampler() const {
            return CustomSampler();
        }
    };

    struct SO3ScenarioWithUniformSampling {
        using Space = SO3Space<double>;
        using State = typename Space::Type;

        Space space() const {
            return Space();
        }
    };

    struct LP2ScenarioWithUniformSampling {
        using Space = L2Space<double, 2>;
        using State = typename Space::Type;
        using Bounds = BoxBounds<double, 2>;

        Bounds bounds_{State(1,2), State(5,9)};

        Space space() const {
            return Space();
        }

        const Bounds& bounds() const {
            return bounds_;
        }
    };

    struct SE3ScenarioWithUniformSampling {
        using Space = SE3Space<double>;
        using State = typename Space::Type;
        using Bounds = CartesianBounds<Unbounded, BoxBounds<double, 3>>;
        using Vec3 = Eigen::Matrix<double, 3, 1>;

        Bounds bounds_{BoxBounds<double, 3>(Vec3(1,2,3), Vec3(5,9,11))};

        Space space() const {
            return Space();
        }

        const Bounds& bounds() const {
            return bounds_;
        }
    };

    struct ScaledSE3ScenarioWithUniformSampling {
        using Space = SE3Space<double, 7, 3>;
        using State = typename Space::Type;
        using Bounds = CartesianBounds<Unbounded, BoxBounds<double, 3>>;
        using Vec3 = Eigen::Matrix<double, 3, 1>;

        Bounds bounds_{BoxBounds<double, 3>(Vec3(1,2,3), Vec3(5,9,11))};

        Space space() const {
            return Space();
        }

        const Bounds& bounds() const {
            return bounds_;
        }
    };
}

TEST(use_scenario_sample_method) {
    using namespace unc::robotics::mpt;

    using Scenario = mpt_test::ScenarioWithSampleMethod;
    using State = typename Scenario::State;
    using RNG = std::mt19937_64;
    using Sampler = impl::scenario_sampler<Scenario, RNG>::type;

    Scenario scenario;
    Sampler sampler(scenario);

    RNG rng;
    State q = sampler(rng);
    EXPECT((q.coeffs() == Eigen::Vector4d(-1,2,3,4).normalized())) == true;
    EXPECT(scenario.sampleCount_) == 1;
}

TEST(use_scenario_sampler_method) {
    using namespace unc::robotics::mpt;

    using Scenario = mpt_test::ScenarioWithSamplerMethod;
    using State = typename Scenario::State;
    using RNG = std::mt19937_64;
    using Sampler = impl::scenario_sampler<Scenario, RNG>::type;

    Scenario scenario;
    Sampler sampler(scenario);

    RNG rng;
    State q = sampler(rng);
    EXPECT((q.coeffs() == Eigen::Vector4d(1,-2,3,4).normalized())) == true;
}

TEST(use_so3_scenario_with_uniform_sampler) {
    using namespace unc::robotics::mpt;

    using Scenario = mpt_test::SO3ScenarioWithUniformSampling;
    using State = typename Scenario::State;
    using RNG = mpt_test::NotRandom<std::uint64_t>;
    using Sampler = impl::scenario_sampler<Scenario, RNG>::type;


    Scenario scenario;
    Sampler sampler(scenario);

    RNG rng;
    State q = sampler(rng);
    // std::cout << q.coeffs() << std::endl;
    EXPECT((q.coeffs() == Eigen::Vector4d(1,0,0,0))) == true;
}

TEST(use_lp2_scenario_with_uniform_sampler) {
    using namespace unc::robotics::mpt;

    using Scenario = mpt_test::LP2ScenarioWithUniformSampling;
    using State = typename Scenario::State;
    using RNG = mpt_test::NotRandom<std::uint64_t>;
    using Sampler = impl::scenario_sampler<Scenario, RNG>::type;


    Scenario scenario;
    Sampler sampler(scenario);

    RNG rng;
    State q = sampler(rng);
    // std::cout << q.coeffs() << std::endl;
    EXPECT((q == Eigen::Vector2d(1,2))) == true;
}

TEST(use_se3_scenario_with_uniform_sampler) {
    using namespace unc::robotics::mpt;

    using Scenario = mpt_test::SE3ScenarioWithUniformSampling;
    using State = typename Scenario::State;
    using RNG = mpt_test::NotRandom<std::uint64_t>;
    using Sampler = impl::scenario_sampler<Scenario, RNG>::type;


    Scenario scenario;
    Sampler sampler(scenario);

    RNG rng;
    State q = sampler(rng);
    // std::cout << q.coeffs() << std::endl;
    EXPECT((std::get<0>(q).coeffs() == Eigen::Vector4d(1,0,0,0))) == true;
    EXPECT((std::get<1>(q) == Eigen::Vector3d(1,2,3))) == true;
}

TEST(use_scaled_se3_scenario_with_uniform_sampler) {
    using namespace unc::robotics::mpt;

    using Scenario = mpt_test::ScaledSE3ScenarioWithUniformSampling;
    using State = typename Scenario::State;
    using RNG = mpt_test::NotRandom<std::uint64_t>;
    using Sampler = impl::scenario_sampler<Scenario, RNG>::type;


    Scenario scenario;
    Sampler sampler(scenario);

    RNG rng;
    State q = sampler(rng);
    // std::cout << q.coeffs() << std::endl;
    EXPECT((std::get<0>(q).coeffs() == Eigen::Vector4d(1,0,0,0))) == true;
    EXPECT((std::get<1>(q) == Eigen::Vector3d(1,2,3))) == true;
}
