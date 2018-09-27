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

#pragma once
#ifndef MPT_IMPL_SCENARIO_SAMPLER_HPP
#define MPT_IMPL_SCENARIO_SAMPLER_HPP

#include "scenario_space.hpp"
#include "scenario_bounds.hpp"
#include "../uniform_sampler.hpp"
#include <type_traits>

namespace unc::robotics::mpt::impl {

    // Checks if Scenario has a sample(RNG&) method
    template <typename Scenario, typename RNG, class = void>
    struct scenario_has_sample_method : std::false_type {};

    template <typename Scenario, typename RNG>
    struct scenario_has_sample_method<Scenario, RNG, std::void_t<decltype( std::declval<Scenario&>().sample(std::declval<RNG&>()) )>>
        : std::true_type {};

    template <typename Scenario, typename RNG>
    constexpr bool scenario_has_sample_method_v = scenario_has_sample_method<Scenario, RNG>::value;

    // Checks if Scenario has a sampler() method
    template <typename Scenario, class = void>
    struct scenario_has_sampler_method : std::false_type {};

    template <typename Scenario>
    struct scenario_has_sampler_method<Scenario, std::void_t<decltype( std::declval<const Scenario&>().sampler() )>>
        : std::true_type {};

    template <typename Scenario>
    constexpr bool scenario_has_sampler_method_v = scenario_has_sampler_method<Scenario>::value;

    // Checks if Scenario has Sampler type
    template <typename Scenario, class = void>
    struct scenario_has_sampler_type : std::false_type {};

    template <typename Scenario>
    struct scenario_has_sampler_type<Scenario, std::void_t<typename Scenario::Sampler>>
        : std::true_type {};

    template <typename Scenario>
    constexpr bool scenario_has_sampler_type_v = scenario_has_sampler_type<Scenario>::value;

    // Bounds:
    //
    //    In the following resolution, bounds is determined by the
    //    existance of a bounds() const method.  If it does not exist,
    //    the bounds are implied as Unbounded (which is fine for
    //    rotational spaces, or custom samplers)
    //
    // Sampler:
    //
    // 1. scenario has a sample(rng) method
    //    i.e., declval<const Scenario&>.sample(rng)
    //
    // 2. scenario has sampler() method which returns an object which
    //    can act as a sampler, i.e., takes an RNG as an argument, and
    //    returns a type.
    //
    // 3. Scenario::Sampler
    //    constructed with space(), and bounds() || Unbounded{}
    //
    // 4. no sample(RNG), sampler(), or Sampler
    //    UniformSampler<Space, Bounds>

    template <typename Scenario, typename Unbounded = void>
    struct ScenarioUniformSampler : UniformSampler<
        scenario_space_t<Scenario>,
        scenario_bounds_t<Scenario>>
    {
        using Base = UniformSampler<
            scenario_space_t<Scenario>,
            scenario_bounds_t<Scenario>>;

        ScenarioUniformSampler(const Scenario& scenario)
            : Base(scenario.space())
        {
        }
    };

    template <typename Scenario>
    struct ScenarioUniformSampler<
        Scenario,
        std::enable_if_t<scenario_has_bounds_v<Scenario>>>
        : UniformSampler<
            scenario_space_t<Scenario>,
            scenario_bounds_t<Scenario>>
    {
        using Base = UniformSampler<
            scenario_space_t<Scenario>,
            scenario_bounds_t<Scenario>>;

        ScenarioUniformSampler(const Scenario& scenario)
            : Base(scenario.space(), scenario.bounds())
        {
        }
    };

    template <typename Scenario, typename RNG, class = void>
    struct scenario_sampler {
        // default: 4. use UniformSampler
        using type = ScenarioUniformSampler<Scenario>;
    };

    template <typename Scenario, typename RNG>
    using scenario_sampler_t = typename scenario_sampler<Scenario, RNG>::type;


    // 1. has sample(RNG&) method
    template <typename Scenario>
    class ScenarioSampleMethodWrapper {
        Scenario& scenario_;
    public:
        ScenarioSampleMethodWrapper(Scenario& scenario)
            : scenario_(scenario)
        {
        }

        template <typename RNG>
        decltype(auto) operator () (RNG& rng) {
            return scenario_.sample(rng);
        }
    };

    template <typename Scenario, typename RNG>
    struct scenario_sampler<
        Scenario, RNG,
        std::enable_if_t<scenario_has_sample_method_v<Scenario, RNG>>>
    {
        using type = ScenarioSampleMethodWrapper<Scenario>;
    };

    // 2. has sampler() method
    template <typename Sampler>
    struct ScenarioSamplerMethodWrapper : Sampler {
        template <typename Scenario>
        ScenarioSamplerMethodWrapper(const Scenario& scenario)
            : Sampler(scenario.sampler())
        {
        }
    };

    template <typename Scenario, typename RNG>
    struct scenario_sampler<
        Scenario, RNG,
        std::enable_if_t<!scenario_has_sample_method_v<Scenario, RNG> &&
                         scenario_has_sampler_method_v<Scenario>>>
    {
        using type = ScenarioSamplerMethodWrapper<
            std::decay_t<decltype( std::declval<const Scenario&>().sampler() )>>;
    };

    // 3. has Sampler type
    // template <typename Scenario>
    // class ScenarioSamplerTypeWrapper : public Scenario::Sampler {
    //     using Base = typename Scenario::Sampler;

    // public:
    //     ScenarioSamplerTypeWrapper(
    //         const Scenario& scenario,
    //         std::enable_if_t<decltype(std::declval<const Scenario&>().bounds())>* = 0)
    //         : Base(scenario.space(), scenario.bounds())
    //     {
    //     }

    //     ScenarioSamplerTypeWrapper(
    //         const Scenario& scenario)
    //         : Base(scenario.space())
    //     {
    //     }
    // };

    template <typename Scenario, typename RNG>
    struct scenario_sampler<
        Scenario, RNG,
        std::enable_if_t<!scenario_has_sample_method_v<Scenario, RNG> &&
                         !scenario_has_sampler_method_v<Scenario> &&
                         scenario_has_sampler_type_v<Scenario>>>
    {
        // TODO: using type = typename Scenario::Sampler;
    };

    // 4. use UniformSampler
}

#endif
