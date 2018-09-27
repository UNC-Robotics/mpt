# Unit Tests for MPT

Running the unit tests requires [ninja](https://ninja-build.org/).  The build system assumes that [Nigh](https://github.com/UNC-Robotics/nigh) is checked out at the same directory level as `mpt`.

    % ./configure.sh
    % ninja

Test classes are in `*_test.cpp` files.  The `./configure.sh` script finds files that match this pattern and generates the build rules to run the tests.  Whenever a new test is added the script will have to be run again.

Some `./configure.sh` script behaviors can be overridden with environment variables.  To set the C++ compiler, use the `CXX` environment variable.  To set the flags the compile will use, set the `CFLAGS` environment variable.   Here are a few examples:

To generate a build file that will use clang++ and compile in debug symbols:

    % CXX=clang++ CFLAGS=-g ./configure.sh

To generate a build file that will use g++ using the default compiler flags:

    % CXX=g++ ./configure.sh

