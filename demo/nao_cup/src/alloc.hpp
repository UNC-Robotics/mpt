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

#ifndef ALLOC_H
#define ALLOC_H

#include <stdlib.h>
#include <pthread.h>


namespace nao_cup {
/* 64 is a fairly standard cache-line size in x86, but this should be
 * parameterized elsewhere. */
#define CACHE_LINE_SIZE 64

#define ALIGN_UP(x, b) (((size_t)(x) + (b-1)) & ~((size_t)(b-1)))
#define CACHE_ALIGN(x) ALIGN_UP(x, CACHE_LINE_SIZE)


static inline int
aligned_offset(void *ptr, size_t b)
{
        size_t address = (size_t)ptr;
        return (ALIGN_UP(address, b) - address);
}




typedef struct tl_mempool {
        pthread_key_t key;
        size_t chunk_size;
} tl_mempool_t;

#define struct_alloc(type) ((type *)malloc(sizeof(type)))
#define array_alloc(type, count) ((type *)malloc(sizeof(type) * count))
#define array_grow(array, count) ((typeof(array))realloc(array, sizeof(array[0]) * count));
#define array_copy(array, count) memdup(array, sizeof(array[0]) * count)

int tl_mempool_init(tl_mempool_t *pool, size_t chunk_size);
int tl_mempool_destroy(tl_mempool_t *pool);
void *tl_alloc(tl_mempool_t *pool, size_t size);
void tl_free(tl_mempool_t *pool, void *ptr);

void *memdup(const void *source, size_t bytes);

}

#endif /* ALLOC_H */
