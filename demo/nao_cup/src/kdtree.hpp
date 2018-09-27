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


#ifndef KD_TREE_H
#define KD_TREE_H

/* for size_t */
#include <stddef.h>
#include <stdint.h>
#include "alloc.hpp"

#ifdef CHECK_CRCS
#include "crc.h"
#endif

namespace nao_cup {

typedef double (*kd_dist_func)(const double *a, const double *b);
typedef void (*kd_near_callback)(void *data, int no, void *value, double dist);

typedef struct kd_tree {
        size_t dimensions;
        const double *min;
        const double *max;
        struct kd_node *root;
        kd_dist_func dist_func;
        tl_mempool_t mempool;
} kd_tree_t;

kd_tree_t *kd_create_tree(size_t dimensions, const double *min, const double *max,
                          kd_dist_func dist_func,
                          const double *root_config, void *root_value);
void kd_insert(kd_tree_t *t, const double *config, void *value);
void *kd_nearest(kd_tree_t *t, const double *target, double *dist);
int kd_near(kd_tree_t *t, const double *target, double radius, kd_near_callback callback, void *cb_data);

}

#endif
