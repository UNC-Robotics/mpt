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

#ifndef PRRTS_H
#define PRRTS_H

#include "kdtree.hpp"

namespace nao_cup {
#define REGION_SPLIT_AXIS 0
#define INITIAL_NEAR_LIST_CAPACITY 1024

    template <typename S>
    using prrts_in_goal_func =  bool (*)(void *system, const S *config);
    template <typename S>
    using prrts_clear_func = bool (*)(void *system, const S *config);
    template <typename S>
    using prrts_link_func = bool (*)(void *system, const S *a, const S *b);
    template <typename S>
    using prrts_system_data_alloc_func =  void *(*)(int thread_no, const S *sample_min, const S *sample_max);

    typedef void (*prrts_system_data_free_func)(void *system);

    template <typename S>
    struct prrts_system {
        size_t dimensions;

        const S *init;
        const S *min;
        const S *max;
        const S *target;

        prrts_system_data_alloc_func<S> system_data_alloc_func;
        prrts_system_data_free_func system_data_free_func;

        kd_dist_func dist_func;
        prrts_in_goal_func<S> in_goal_func;
        prrts_clear_func<S> clear_func;
        prrts_link_func<S> link_func;
    };

// typedef struct prrts_options {
//         double gamma;
//         bool regional_sampling;
//         int samples_per_step;
// } prrts_options_t;

// typedef struct prrts_solution {
//         double path_cost;
//         size_t path_length;
//         const double *configs[0];
// } prrts_solution_t;

// prrts_solution_t* prrts_run_for_duration(prrts_system_t *system, prrts_options_t *options, int thread_count, long duration);
// prrts_solution_t* prrts_run_for_samples(prrts_system_t *system, prrts_options_t *options, int thread_count, size_t sample_count);
// prrts_solution_t* prrts_run_indefinitely(prrts_system_t *system, prrts_options_t *options, int thread_count);

}

#endif /* PRRTS_H */
