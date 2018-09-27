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

#include <mpt/log.hpp>
#include <thread>

// #include <cstdio>

// #define _SELECT(PREFIX,_5,_4,_3,_2,_1,SUFFIX, ...) PREFIX ## _ ## SUFFIX

// // #define _LOG_1(L)          printf("%s:%d: [" #L "] (no message)\n", __FILE__, __LINE__)
// #define _LOG_1(L)          printf("[%s] (no message)\n", #L)
// #define _LOG_2(L,fmt)      printf("[%s] %s\n", #L, fmt)
// #define _LOG_N(L,fmt, ...) printf("[%s] " fmt "\n", #L, __VA_ARGS__)
// #define LOG(...) _SELECT(_LOG,__VA_ARGS__,N,N,N,2,1)(__VA_ARGS__)

// #define _BAR_1(fmt)      printf(fmt "\n")
// #define _BAR_N(fmt, ...) printf(fmt "\n", __VA_ARGS__);
// #define BAR(...) _SELECT(_BAR,__VA_ARGS__,N,N,N,N,1)(__VA_ARGS__)

int main(int argc, char *argv[]) {
    std::thread thread([&] {
            MPT_LOG(INFO) << "this is not the main thread"; });

    thread.join();
    MPT_LOG(TRACE) << "a trace message with number " << 42.0;
    MPT_LOG(DEBUG) << "hello";
    MPT_LOG(INFO) << "this is info";
    MPT_LOG(WARN) << "achtung";
    MPT_LOG(ERROR) << "an error";
    MPT_LOG(FATAL) << "fatal error";
    MPT_LOG(INFO, "%s world %f %d %d", "three", 4.0, 5, 6);
    MPT_LOG(WARN, "message without arguments (%s and %d, etc... are ignored)");

    if (true)
        MPT_LOG(TRACE) << "dangling else warning?";

    // //LOG();
    // //                    PREFIX,_5,_4,_3,_2,_1,SUFFIX
    // // expands to _SELECT(_LOG   ,  , N, N, N, N,     1)()
    // //  (note, empty parameter _5 is OK in macros)
    // // expands to    LOG ## _ ## 1 ()
    // //               _LOG_1 ()

    // // LOG(); // this does NOT work, because it expands to _LOG_1() => printf("%s:%d: %s\n, __FILE__, __LINE__, );
    // LOG(INFO);
    // LOG(DEBUG, "here is a log message");
    // LOG(WARN, "here is a log message with param: %d", 42);

    // BAR(); // this works, because it expands to _SELECT(_BAR,,N,N,N,N,1)() => _BAR_1() => printf("\n")
    // BAR("here is a log message");
    // BAR("here is one with a param: %d", 42);

    // BAR("here is a log message");
    // BAR("here is a log message with a param: %d", 42);

    return 0;
}
