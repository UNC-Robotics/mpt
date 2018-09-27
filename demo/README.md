# MPT Demos

This directory contains demos for MPT.

The build process for each of the scenarios create variants for `float` and `double` scalar precision, `kd-tree` and `GNAT` nearest neighbors, and single- (`st`) and multi-threaded (`mt`).  By default the multithreaded versions will use all hardware parallelism available.  Set the environment variable `OMP_NUM_THREADS` to a number to specify exactly how many threads the multithreaded planners should use.

The planner executable accept the following arguments:

* `-a ALGORITHM` solve using specified algorithm, currently supports `rrt` and `rrtstar`.
* `-S` run until solved
* `-t N` run for N milliseconds
* `-n N` run until graph contains N nodes

## Nao Robot 10 DOF Motion Planning

The scenario in `nao_cup_planning.cpp` demonstrates using MPT to plan motions for the [SoftBank Nao Robot](https://www.softbankrobotics.com/emea/en/robots/nao).  In this planning scenario, the Nao robot starts with a cup of water in one hand and a effervescent tablet in the other, and the robot moves its arms to drop the tablet into the cup without hitting and obstacle or spilling the water in the process.  Each arm as 5 degrees of freedom (DOF) so the motion plan has a total of 10 DOF.


## SE(3) Rigid Body Motion Planning

This planner loads and solves [OMPL.app](http://ompl.kavrakilab.org/gui.html)'s rigid body planning scenarios.  If you haven't already, downloaded and try out OMPL!  To use these demos specify the name of the configuration file on the command line, for example, to solve (`-S`) the alpha-1.5 problem using RRT (`-a rrt`), use:

     build/se3_rigid_body_planning-kd-double-mt path-to-ompl/resources -S -a rrt path-to-omplapp/resources/3D/alpha-1.5.cfg

Note: this demo shows MPT's capabilities and can be used to compare between algorithms within MPT.  It should NOT be used to compare between OMPL and MPT.  There are a number of difference between OMPL and MPT making benchmarking OMPL vs MPT through this inaccurate and inappropriate.  To name a few differences: interpolation during collision detection, sampling approaches, algorithm constants and defaults, and well as basic algorithm structures.  To do a fair comparison, one would have to control for all these factors.

# Requirements

* C++ 17 compiler (such as [GCC 8](https://gcc.gnu.org/) or [clang 6](https://clang.llvm.org/))
* The [ninja](https://ninja-build.org/) build system
* [Eigen 3](http://eigen.tuxfamily.org) linear algebra library
* [Nigh](https://github.com/UNC-Robotics/nigh) concurrent nearest neighbors library
* [FCL](https://github.com/flexible-collision-library/fcl) (and transitive dependency [CCD](https://github.com/danfis/libccd)) collision detection
* [The Open Asset Importer](http://www.assimp.org/)

## Install Dependencies on Ubuntu

    sudo apt install ninja eigen3 libfcl libassimp

Make sure you have a build environment with the latest compilers:

    sudo apt install build-essential g++-8 gcc-8

(Optional) You may also want to try clang++ instead of g++, as it tends to give better error messages:

    sudo apt install clang-6.0

Unfortunately older variants of Ubuntu do not tend to include up-to-date compilers.  MPT uses C++ 17 (e.g. 2017 standard), which came out after Ubuntu 16 (2016 version).  If you are stuck with Ubuntu 16 or earlier, additional work make be required to install a C++ 17 compiler.

## Install Dependencies on OSX with MacPorts

    sudo port install ninja eigen3 ccd assimp

The clang included with OSX/Xcode may be based upon an older version of clang and not work.  Install the latest version from macports, and 'select' it as the default:

    sudo port install clang-6.0
    sudo port select clang mp-clang-6.0
    clang++ --version # should output 'clang version 6.0...'

Alternately, use a GNU compiler:

    sudo port install gcc8
    sudo port select gcc mp-gcc8
    g++ --version # should output 'g++ (MacPorts gcc8 ...'

## Install Dependencies on OSX with HomeBrew

This homebrew is not tested, but the following (or something similar is likely to work).  Feedback on this section is welcome!

    brew tap ninja eigen3 ccd assimp

# Building

The build is a two-step process. The first step generates a build script for ninja, and the second builds everything with ninja.  The result of the build is placed in the `build` subfolder.

    ./configure.sh
    ninja

To use a different C++ compiler, define the `CXX` variable before running `./configure.sh`, e.g.,

    CXX=clang++ ./configure.sh

# Troubleshooting

If your compiler complains about OpenMP, omp, or similar, use:

    CFLAGS="-fopenmp -O3 -march=native" ./configure.sh

This may be combined with CXX setting, e.g.

    CXX=clang++ CFLAGS="-fopenmp -O3 -march=native" ./configure.sh

