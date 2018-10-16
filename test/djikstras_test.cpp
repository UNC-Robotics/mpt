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

#include <mpt/impl/djikstras.hpp>
#include <forward_list>
#include "test.hpp"

namespace mpt_test {
    struct Vertex {
        int id_;
        std::forward_list<std::pair<double, Vertex*>> edges_;

        explicit Vertex(int id) : id_(id) {}

        void addEdge(double w, Vertex& v) {
            edges_.emplace_front(w, &v);
            v.edges_.emplace_front(w, this);
        }
    };
}

TEST(find_shortest_path) {
    std::size_t N = 7;
    std::vector<mpt_test::Vertex> V;
    V.reserve(N);
    for (std::size_t i=0 ; i<N ; ++i)
        V.emplace_back(i);

    V[0].addEdge(1.0, V[1]);
    V[1].addEdge(2.0, V[3]); V[0].addEdge(3.1, V[3]);
    V[2].addEdge(4.0, V[4]); V[0].addEdge(10.3, V[4]);
    V[3].addEdge(3.0, V[2]); V[0].addEdge(6.2, V[2]);
    
    V[0].addEdge(1.5, V[5]);
    V[5].addEdge(9.5, V[6]);
    V[6].addEdge(9.9, V[4]);

    V[1].addEdge(7.0, V[2]);

    std::vector<int> result = unc::robotics::mpt::impl::djikstras<mpt_test::Vertex*, double>(
        &V[0],
        // Goal check, V[4] is our goal
        [&] (const auto& v) { return v == &V[4]; },
        // Edge callback
        [&] (const auto& v, auto callback) {
            for (auto [w, u] : v->edges_)
                callback(w, u);
        },
        // results callback
        [&] (std::size_t n, auto first, auto last) {
            EXPECT(n) == 5;
            auto it = first;
            EXPECT((*it++)->id_) == 0;
            EXPECT((*it++)->id_) == 1;
            EXPECT((*it++)->id_) == 3;
            EXPECT((*it++)->id_) == 2;
            EXPECT((*it++)->id_) == 4;
            std::vector<int> result;
            result.reserve(n);
            for (auto it = first ; it != last ; ++it)
                result.push_back((*it)->id_);
            return result;
        });

    EXPECT(result.size()) == 5;
    EXPECT(result[0]) == 0;
    EXPECT(result[1]) == 1;
    EXPECT(result[2]) == 3;
    EXPECT(result[3]) == 2;
    EXPECT(result[4]) == 4;
}

