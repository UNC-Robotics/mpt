#!/bin/bash

die () { echo "FATAL: $1" ; exit 1; }

NINJA="${NINJA:-`which ninja`}" || die "ninja build tool not found"
echo "Ninja found at $NINJA"

CXX="${CXX:-`which c++`}" || CXX=`which clang++` || CXX=`which g++` || die "C++ compile not found"
echo "C++ compiler found at $CXX"

DEPS="eigen3 fcl assimp"
CFLAGS="${CFLAGS:--O3 -march=native}"
CFLAGS+=" -std=c++17 -I../src -I../../nigh/src"

set -o pipefail
CXX_VERSION=$($CXX --version | head -n 1) || die "C++ compiler not found"
case $CXX_VERSION in
    
    g++*)
        # g++ (...) X.Y.Z
        # g++ (...) X.Y.Z 20171010
        GCC_VERSION=${CXX_VERSION##*) } # remove longest prefix before ') '
        echo "GNU C++ compiler $GCC_VERSION found at $CXX"
        case $GCC_VERSION in
            [0-4].*) echo "WARNING: compiler version too old.  Build will fail." ;;
            [56].*) echo "WARNING: compiler version too old.  Build may fail." ;;
            [789].*) ;;
            [1-9][0-9].*) ;;
            *) echo "WARNING: unknown version.  Build may fail." ;;
        esac
        CFLAGS+=" -fopenmp"
        ;;
    Apple*)
        # Apple LLVM version 10.0.0 (clang-1000.11.45.2)
        CLANG_VERSION=${CXX_VERSION#*version }
        CLANG_VERSION=${CLANG_VERSION%% *}
        echo "Apple LLVM C++ compiler $CLANG_VERSION found at $CXX"
        if [[ $CLANG_VERSION = [0-9].* ]] ; then
            echo "WARNING: compiler version too old.  Build may fail."
        fi
        if [[ `uname -r` = 1[0-7].* ]] ; then
            # it would be nice to figure this out... it should be
            # possible but compilation fails at points warning about
            # std c++ libraries not being available until 10.14
            echo "WARNING: build may fail without macOS Mohave (10.14) or later"
        fi
        CFLAGS+=" -Xpreprocessor -fopenmp"
        LIBS+=" -lomp"
        ;;
    clang*)
        # clang version X.Y.Z (...)
        CLANG_VERSION=${CXX_VERSION#*version } # remove prefix up to 'version'
        CLANG_VERSION=${CLANG_VERSION%% *} # remove suffix after space
        echo "LLVM C++ compiler $CLANG_VERSION found at $CXX"
        if [[ $CLANG_VERSION = [0-4].* ]] ; then
            echo "WARNING: compiler version too old.  Build will fail."
        fi
        # MacPort's libomp does not install to /opt/local/include/omp.h without the +top_level variant
        # it also does not provide a pkg-config
        CFLAGS+=" -fopenmp"
        LIBS+=" -lomp"
        ;;
    *)
        echo "WARNING: unknown compiler.  Build may fail."
        CFLAGS+=" -fopenmp" # cross-fingers, hope it works!
        ;;
esac

PKG_CONFIG="${PKG_CONFIG:-`which pkg-config`}" || die "pkg-config not found"
echo "pkg-config found at $PKG_CONFIG"


VERSIONS=$($PKG_CONFIG --modversion $DEPS) || die "required dependency is missing"

DEPS_ALL=$((printf "%s\n" $DEPS ; $PKG_CONFIG --print-requires $DEPS) | sort | uniq)

CFLAGS+=" $($PKG_CONFIG --cflags-only-I $DEPS_ALL)"
LIBS="$($PKG_CONFIG --libs eigen3 $DEPS_ALL)"

case `uname` in
    Linux)
        LIBS+=" -lpthread"
    ;;
esac

echo "Required dependencies found, generating build.ninja"
exec 3> build.ninja # open build.ninja for writing, use descriptor 3

cat <<-EOF >&3
	ninja_required_version = 1.3
	root = .
	builddir = build
	cxx = $CXX
	cflags = $CFLAGS
	libs = $LIBS

	rule cxx
	  command = \$cxx -MMD -MT \$out -MF \$out.d \$cflags -DNN_TYPE=\$nn -DSCALAR_TYPE=\$scalar -DMT=\$mt \$in -o \$out \$libs
	  description = CXX \$out
	  depfile = \$out.d
	  deps = gcc

	EOF

nao_variants=""
se3_variants=""
for scalar in float double ; do
    for nn in kd gnat ; do
        for t in st mt ; do
            nao_variants+=" \$builddir/nao_cup_planning-$nn-$scalar-$t"
            se3_variants+=" \$builddir/se3_rigid_body_planning-$nn-$scalar-$t"
            [[ $nn = kd ]] && NN="KDTreeBatch" || NN="GNAT"
            [[ $t = st ]] && MT=0 || MT=1
            cat <<-EOF >&3
		build \$builddir/nao_cup_planning-$nn-$scalar-$t: cxx nao_cup_planning.cpp
		  nn = $NN
		  scalar = $scalar
		  mt = $MT

		build \$builddir/se3_rigid_body_planning-$nn-$scalar-$t: cxx se3_rigid_body_planning.cpp
		  nn = $NN
		  scalar = $scalar
		  mt = $MT

		EOF
        done
    done
done

cat <<-EOF >&3
build nao_cup_planning: phony $nao_variants
build se3_rigid_body_planning: phony $se3_variants
build all: phony nao_cup_planning se3_rigid_body_planning
EOF

exec 3>&- # close build.ninja
echo "Done.  type 'ninja' to start build"
