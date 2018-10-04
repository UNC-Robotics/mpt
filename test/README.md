# Unit Tests for MPT

Running the unit tests requires [CMake](https://cmake.org/). The build assumes that [Nigh](https://github.com/UNC-Robotics/nigh) is checked out at the same directory level as `mpt`.  Use the following commands to build (we assume the ninja build system is available):

    % mkdir build
    % cmake -G Ninja ..
    % ninja

## Troubleshooting

### Xcode + macOS + C++17

The compiler included with macOS's Xcode does not appear to support all of C++17 without first upgrading to Mohave (macOS 10.14).  Because there are also issues with getting OpenMP to work with this compiler, we recommend using a different compiler.  See next tip.

### Xcode + macOS + OpenMP

The compiler included with macOS's Xcode supports OpenMP, but requires extra flags that CMake doesn't appear to be able to add.  We recommend using a custom install of `clang` (version 5+) or `gcc` (version 7+)  and `libomp` using either MacPorts or Homebrew. Then make sure that cmake uses the right compiler:

    % mkdir build
    % cd build
    % CXX=/path/to/clang++ CC=/path/to/clang cmake -G Ninja ..
    % ninja

