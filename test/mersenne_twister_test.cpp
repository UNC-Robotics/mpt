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

#include "test.hpp"
#include <random>
#include <algorithm>
#include <mpt/mersenne_twister.hpp>


template <typename _STD, typename _MPT>
void testSameSequence(_STD& mtstd, _MPT& mtmpt) {
    EXPECT(_STD::min()) == _MPT::min();
    EXPECT(_STD::max()) == _MPT::max();

    for (int i=0 ; i<10000 ; ++i)
        EXPECT(mtmpt()) == mtstd();
    mtmpt.discard(9999);
    mtstd.discard(9999);
    for (int i=0 ; i<10000 ; ++i)
        EXPECT(mtmpt()) == mtstd();

}

TEST(SameSequenceWithDefaultSeed32) {
    std::mt19937 mtstd;
    unc::robotics::mpt::MersenneTwister19937_32 mtmpt;
    testSameSequence(mtstd, mtmpt);
}

TEST(SameSequenceWithDefaultSeed64) {
    std::mt19937_64 mtstd;
    unc::robotics::mpt::MersenneTwister19937_64 mtmpt;
    testSameSequence(mtstd, mtmpt);
}

TEST(SameSequenceWithSeedArgument32) {
    std::mt19937 mtstd(123456789);
    unc::robotics::mpt::MersenneTwister19937_32 mtmpt(123456789);
    testSameSequence(mtstd, mtmpt);
}

TEST(SameSequenceWithSeedArgument64) {
    std::mt19937_64 mtstd(123456789);
    unc::robotics::mpt::MersenneTwister19937_64 mtmpt(123456789);
    testSameSequence(mtstd, mtmpt);
}

TEST(SameSequenceWithSeedSequence32) {
    std::seed_seq seq0{1,2,3,4,5,6,7,8,9,10};
    std::mt19937 mtstd(seq0);
    std::seed_seq seq1{1,2,3,4,5,6,7,8,9,10};
    unc::robotics::mpt::MersenneTwister19937_32 mtmpt(seq1);
    testSameSequence(mtstd, mtmpt);
}

TEST(SameSequenceWithSeedSequence64) {
    std::seed_seq seq0{1,2,3,4,5,6,7,8,9,10};
    std::mt19937_64 mtstd(seq0);
    std::seed_seq seq1{1,2,3,4,5,6,7,8,9,10};
    unc::robotics::mpt::MersenneTwister19937_64 mtmpt(seq1);
    testSameSequence(mtstd, mtmpt);
}

TEST(serializeDeseriaize) {
    typedef unc::robotics::mpt::MersenneTwister19937_32 RNG;

    std::ostringstream ostr;
    RNG rng0;
    rng0.discard(1);
    ostr << rng0;

    RNG rng1{2};
    rng1.discard(7);
    std::istringstream istr(ostr.str());

    EXPECT(rng0 == rng1) == false;

    istr >> rng1;

    EXPECT(rng0 == rng1) == true;

    std::ostringstream checkStr;
    checkStr << rng1;

    EXPECT(ostr.str()) == checkStr.str();

    //std::cout << ostr.str() << std::endl;
    testSameSequence(rng0, rng1);
}

// The following code is the start of something to check the
// performance of our mersenne twister relative to STD.  In
// particular, we're looking to get lower variance.  However due to
// the fast performance of both we run into problems where a small
// variation (e.g., due to external factors) can result in a huge
// change in the result.  On average however, our version appears to
// have slightly better performace and much lower variance.

// #include "linuxclock.hpp"

// struct Stats {
//     double mean_;
//     double variance_;

//     template <typename _T, typename _A>
//     Stats(const std::vector<_T, _A>& values)
//         : mean_(std::accumulate(values.begin(), values.end(), 0.0) / values.size())
//     {
//         double varSum = 0;
//         for (_T x : values)
//             varSum += (x - mean_) * (x - mean_);

//         variance_ = std::sqrt(varSum / values.size());
//     }
// };

// template <typename _RNG>
// // std::pair<std::uint64_t, std::uint64_t>
// void checkTimings(std::size_t N) {
//     _RNG rng;

//     //typedef std::chrono::high_resolution_clock Clock;
//     typedef ClockGetTime<CLOCK_MONOTONIC> Clock;

//     typedef std::chrono::nanoseconds::rep nanoint;

//     double sum = 0, sum2 = 0;
//     nanoint min = std::numeric_limits<nanoint>::max(), max = 0;
//     for (std::size_t i = 0 ; i<N ; ++i)
//         rng();
//     for (std::size_t i = 0 ; i<N ; ++i) {
//         auto start = Clock::now();
//         rng();
//         auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
//             Clock::now() - start).count();

//         sum += elapsed;
//         sum2 += elapsed*elapsed;
//         max = std::max(max, elapsed);
//         min = std::min(min, elapsed);
//     }

//     //Stats stats(timings);

//     //std::cout << stats.mean_ << " " << stats.variance_ << std::endl;
//     std::cout << sum / N << " " << min << " " << max << " " << std::sqrt((sum2 - sum*sum/N)/N) << std::endl;
// }

// TEST(ConsistentTiming) {
//     std::size_t N = 624*100;
//     for (int i=0 ; i<3 ; ++i) {
//         std::cout << "std:: ";
//         checkTimings<std::mt19937_64>(N);
//         std::cout << "mpt:: ";
//         checkTimings<unc::robotics::mpt::MersenneTwister19937_64>(N);
//     }
// }

