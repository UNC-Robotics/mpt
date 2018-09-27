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

#ifndef MPT_MERSENNE_TWISTER_HPP_
#define MPT_MERSENNE_TWISTER_HPP_

#include <limits>
#include <cassert>
#include <array>

namespace unc::robotics::mpt {


    template <typename _UInt, std::size_t _w,
              std::size_t _n, std::size_t _m, std::size_t _r,
              _UInt _a, std::size_t _u, _UInt _d, std::size_t _s,
              _UInt _b, std::size_t _t,
              _UInt _c, std::size_t _l, _UInt _f>
    class MersenneTwister {
        static_assert(std::is_unsigned<_UInt>::value,
                      "type must be an unsigned integer type");
        static_assert(_w == 32 || _w == 64, "n must be a power of 2"); // avoids mod/shift issues
        static_assert(_w <= std::numeric_limits<_UInt>::digits,
                      "w is too large for result type");

    public:
        typedef _UInt result_type;

        // parameter values, these are provided for consistency with
        // std::mersenne_twister, but it's not clear why this is
        // important.
        static constexpr std::size_t word_size                 = _w;
        static constexpr std::size_t state_size                = _n;
        static constexpr std::size_t shift_size                = _m;
        static constexpr std::size_t mask_bits                 = _r;
        static constexpr result_type xor_mask                  = _a;
        static constexpr std::size_t tempering_u               = _u;
        static constexpr result_type tempering_d               = _d;
        static constexpr std::size_t tempering_s               = _s;
        static constexpr result_type tempering_b               = _b;
        static constexpr std::size_t tempering_t               = _t;
        static constexpr result_type tempering_c               = _c;
        static constexpr std::size_t tempering_l               = _l;
        static constexpr result_type initialization_multiplier = _f;
        static constexpr result_type default_seed              = 5489u;

    private:
        static constexpr _UInt upperMask_ = (~_UInt()) << mask_bits;

        std::array<_UInt, state_size> state_;
        std::size_t    offset_;

        static constexpr _UInt maskWord(_UInt x) {
            if constexpr (word_size < std::numeric_limits<_UInt>::digits)
                return x & ((_UInt(1) << word_size) - 1);
            else
                return x;
            // return (word_size < std::numeric_limits<_UInt>::digits
            //         ? x & ((_UInt(1) << word_size) - 1) : x);
        }

        __attribute__((always_inline))
        void advance() {
            constexpr _UInt lowerMask = ~upperMask_;

            std::size_t i = offset_;
            assert(i < state_size);
            if (++offset_ == state_size)
                offset_ = 0;

            // typical implementations of mersenne twister will
            // perform the next update in a batched O(n) operation.
            // Presumably this saves a little time somehow, but here
            // we prefer to generate random numbers at a consistent
            // rate.  So we perform a single mersenne twister state
            // update as soon as we've extracted a result.
            _UInt y = ((state_[i] & upperMask_) | (state_[offset_] & lowerMask));
            state_[i] = (state_[(i + shift_size) % state_size] ^ (y>>1) ^ ((y&1)?xor_mask : 0));
        }

    public:

        // constructors and member function
        explicit MersenneTwister(result_type s = default_seed) {
            seed(s);
        }

        /**
         * @brief Constructs a %mersenne_twister_engine random number generator
         *        engine seeded from the seed sequence @p __q.
         *
         * @param __q the seed sequence.
         */
        template<typename _Sseq, typename = typename
                 std::enable_if<!std::is_same<_Sseq, MersenneTwister>::value>
                 ::type>
        explicit MersenneTwister(_Sseq& sseq) {
            seed(sseq);
        }

        void seed(result_type s = default_seed) {
            state_[0] = maskWord(s);
            for (std::size_t i = 1; i < state_size; ++i) {
                _UInt x = state_[i - 1];
                x ^= x >> (word_size - 2);
                x *= initialization_multiplier;
                x += i % state_size;
                state_[i] = maskWord(x);
            }
            offset_ = 0;
            discard(state_size);
        }

        template<typename _Sseq>
	typename std::enable_if<std::is_class<_Sseq>::value>::type
        seed(_Sseq& sseq) {
            static constexpr std::size_t wordCount = (word_size + 31) / 32;
            uint_least32_t seedData[state_size * wordCount];
            sseq.generate(seedData + 0, seedData + state_size * wordCount);

            bool allZeros = true;
            for (std::size_t i = 0; i < state_size; ++i) {
                _UInt factor = 1u;
                _UInt sum = 0u;
                for (std::size_t j = 0; j < wordCount; ++j) {
                    sum += seedData[wordCount * i + j] * factor;
		    if constexpr (32 < std::numeric_limits<_UInt>::digits)
			 factor <<= 32;
                }
                state_[i] = maskWord(sum);

                if (allZeros) {
                    if (i == 0) {
                        if ((state_[0] & upperMask_) != 0u)
                            allZeros = false;
                    } else if (state_[i] != 0u)
                        allZeros = false;
                }
            }
            if (allZeros)
                state_[0] = _UInt(1) << (word_size - 1);
            offset_ = 0;
            discard(state_size);
        }

        /**
         * @brief Gets the smallest possible value in the output range.
         */
        static constexpr result_type min() { return 0; };

        /**
         * @brief Gets the largest possible value in the output range.
         */
        static constexpr result_type max() {
            // TODO: occasionally clang produces this warning.  Investigate and fix:

// mersenne_twister.hpp:150:33: warning: shift count >= width of type
//      [-Wshift-count-overflow]
//                    ? (_UInt(1) << word_size) - 1
//                                ^  ~~~~~~~~~
// /usr/lib/gcc/x86_64-linux-gnu/7.2.0/../../../../include/c++/7.2.0/bits/uniform_int_dist.h:232:36: note: in instantiation of
//      member function 'unc::robotics::mpt::MersenneTwister<unsigned long, 64, 312, 156, 31, 13043109905998158313, 29,
//      6148914691236517205, 17, 8202884508482404352, 37, 18444473444759240704, 43, 6364136223846793005>::max' requested here
//        const __uctype __urngmax = __urng.max();

            if constexpr (word_size < std::numeric_limits<_UInt>::digits) {
                return (_UInt(1) << word_size) - 1;
            } else {
                return std::numeric_limits<_UInt>::max();
            }

            // return (word_size < std::numeric_limits<_UInt>::digits
            //         ? (_UInt(1) << word_size) - 1
            //         : std::numeric_limits<_UInt>::max());
        }

        /**
         * @brief Discard a sequence of random numbers.
         */
        void discard(unsigned long long n) {
            for ( ; n>0 ; --n)
                advance();
        }

        result_type operator()() {
            std::size_t i = offset_;
            assert(i < state_size);
            result_type z = state_[i];

            // std::cout << "[" << i << "] = " << z << std::endl;
            z ^= (z >> tempering_u) & tempering_d;
            z ^= (z << tempering_s) & tempering_b;
            z ^= (z << tempering_t) & tempering_c;
            z ^= (z >> tempering_l);

            advance();

            return z;
        }

        /**
         * @brief Compares two % mersenne_twister_engine random number generator
         *        objects of the same type for equality.
         *
         * @param lhs A % mersenne_twister_engine random number generator
         *              object.
         * @param rhs Another % mersenne_twister_engine random number
         *              generator object.
         *
         * @returns true if the infinite sequences of generated values
         *          would be equal, false otherwise.
         */
        friend bool operator==(
            const MersenneTwister& lhs,
            const MersenneTwister& rhs)
        {
            // this is not pedantically correct.  We can have two
            // MersenneTwister objects with shifted state buffers and
            // correspondingly shifted offsets such that they will
            // producethe same sequence.  This is extremely unlikely
            // to occur in practice however, unless specifically set
            // up that way.
            return lhs.offset_ == rhs.offset_ &&
                std::equal(lhs.state_.begin(), lhs.state_.end(), rhs.state_.begin());
        }

        template<typename _CharT, typename _Traits>
	friend std::basic_ostream<_CharT, _Traits>& operator<<(
            std::basic_ostream<_CharT, _Traits>& os,
            const MersenneTwister& x)
        {
            typedef std::basic_ostream<_CharT, _Traits> ostream_type;
            typedef typename ostream_type::ios_base ios_base;
            const typename ios_base::fmtflags flags = os.flags();
            const _CharT fill = os.fill();
            const _CharT space = os.widen(' ');
            os.flags(ios_base::dec | ios_base::fixed | ios_base::left);
            os.fill(space);
            for (std::size_t i = 0; i < state_size; ++i)
                os << x.state_[i] << space;
            os << x.offset_;
            os.flags(flags);
            os.fill(fill);
            return os;
        }

        template<typename _CharT, typename _Traits>
	friend std::basic_istream<_CharT, _Traits>& operator>>(
            std::basic_istream<_CharT, _Traits>& is,
            MersenneTwister& x)
        {
            typedef std::basic_istream<_CharT, _Traits> istream_type;
            typedef typename istream_type::ios_base ios_base;

            const typename ios_base::fmtflags flags = is.flags();
            is.flags(ios_base::dec | ios_base::skipws);
            for (std::size_t i = 0; i < state_size; ++i)
                is >> x.state_[i];
            is >> x.offset_;
            is.flags(flags);
            return is;
        }
    };

    // We could use the impl versions directly, but this creates very
    // long class names in debugging and error messages.
    namespace impl {
        using MersenneTwister19937_32 = MersenneTwister<
            std::uint_fast32_t,
            32, 624, 397, 31,
            0x9908b0dfUL, 11,
            0xffffffffUL, 7,
            0x9d2c5680UL, 15,
            0xefc60000UL, 18, 1812433253UL>;

        using MersenneTwister19937_64 = MersenneTwister<
            std::uint_fast64_t,
            64, 312, 156, 31,
            0xb5026f5aa96619e9ULL, 29,
            0x5555555555555555ULL, 17,
            0x71d67fffeda60000ULL, 37,
            0xfff7eee000000000ULL, 43,
            6364136223846793005ULL>;
    }

    struct MersenneTwister19937_32 : impl::MersenneTwister19937_32 {
        using impl::MersenneTwister19937_32::MersenneTwister19937_32;
    };

    struct MersenneTwister19937_64 : impl::MersenneTwister19937_64 {
        using impl::MersenneTwister19937_64::MersenneTwister19937_64;
    };

    template <typename Scalar>
    using mersenne_twister_select = std::conditional_t<
        (sizeof(Scalar) <= 4),
        MersenneTwister19937_32,
        MersenneTwister19937_64>;
}

#endif // MPT_MERSENNE_TWISTER_HPP_
