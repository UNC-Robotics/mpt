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

#ifndef MPT_DEMO_PROC_INFO_HPP
#define MPT_DEMO_PROC_INFO_HPP

#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iterator>

#ifdef linux
#include <malloc.h>
#endif

namespace mpt_demo {
    class ProcInfo {
    public:
	template <typename Char, typename Traits>
	friend decltype(auto) operator<<(
	    std::basic_ostream<Char,Traits>& out, const ProcInfo& p)
	{

            // TODO: need to find OSX equivalent
#ifdef linux
            struct mallinfo mi = mallinfo();

            // std::cout << std::flush;
            // std::cerr << "==== malloc_stats ===== " << std::endl;
            // malloc_stats();
            // std::cerr << "==== malloc_stats end =====" << std::endl;

            out <<
                // arena is also called "system bytes"
                // and is equivalent to mi.uordblks + mi.fordblks
                "Non-mmapped space allocated (bytes): " << mi.arena << "\n"
                "Space allocated in mmapped regions (bytes): " << mi.hblkhd << "\n"
                "Maximum total allocated space (bytes): " << mi.usmblks << "\n"
                "Total allocated space (bytes): " << mi.uordblks << "\n"
                "Total free space (bytes): " << mi.fordblks << "\n"
                "Top-most, releasable space (bytes): " << mi.keepcost << "\n";
#endif

#if 0
            pid_t pid_{getpid()};
	    std::string fileName =
		"/proc/" + std::to_string(pid_) + "/status";
	
	    std::ifstream fin(fileName);

	    if (!fin)
		throw std::runtime_error("failed to open: "+fileName);

	    fin.unsetf(std::ios_base::skipws);
	
	    std::copy(
		std::istreambuf_iterator<char>(fin),
		std::istreambuf_iterator<char>(),
		std::ostream_iterator<Char>(out));
#endif	
	    return out;
	}
    };
}

#endif
