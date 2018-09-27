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
#ifndef MPT_SCENARIO_HPP
#define MPT_SCENARIO_HPP

namespace unc::robotics::mpt {
    template <
        typename Derived,
        typename Space,
        typename ... Options>
    class Scenario;

    // base case
    template <
        typename Derived,
        typename Space_>
    class Scenario {
    public:
        using Space = Space_;
        using State = typename Space::State;

    private:
        Space space_;
    public:
        Scenario(const Space& space = Space())
            : space_(space)
        {
        }

        const Space& space() const {
            return space_;
        }
    };

    template <
        typename Derived,
        typename Space,
        typename ... Options>
    class Scenario<Derived, Space, tag::discrete_motion_validator, Options...>
        : public Scenario<Derived, Space, Options...>
    {
        using Base = Scenario<Derived, Space, Options...>;

        using Valid = std::result_of_t<std::mem_fn(&Derived::valid)>;

        DiscreteMotionValidator<Space, Valid> link_{Base::space(), std::mem_fn(&Derived::valid)};

    public:
        using State = typename Base::State;

        using Base::Base;

        bool link(const State& a, const State& b) const {
            return link_(a, b);
        }
    };


#if 0
    template <typename Scalar>
    class ExampleScenario : public Scenario<
        ExampleScenario<Scalar>,
        SE3Space<Scalar>,
        tag::uniform_sampler,
        tag::discrete_motion>
    {
        using Space = SE3Space<Scalar>;
        using State = typename Space::Space;

        Space space_;
        mpt::DiscreteMotionValidator<Space, ExampleScenario> link_{*this};
        mpt::GoalState<Space> goal_{space_, radius};

    public:
        // Scenario must provide a copy constructor
        const Space& space() const;

        template <typename RNG>
        State sample(RNG& rng);
        // std::optional<State> - if sampler can fail

        bool valid(const State& q);
        // std::optional<State> - if validation can generate a nearby state that is valid

        bool link(const State& a, const State& b) { return link_(a, b); }
        // std::optional<State> - if link can return a different state close to b
        // std::optional<Trajectory> - if the planner is to track trajectories between states
        // std::optional<std::pair<Trajectory, State>> - combination of above two

        bool goal(const State& q);
        // std::optional<State> - if goal check can find a nearby goal
        // ??? std::optional<Trajectory>
        // std::optional<std::pair<Trajectory, State>>

    };
#endif
}

#endif
