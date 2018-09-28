#!/bin/bash

die () { echo "FATAL: $1" ; exit 1; }

NINJA="${NINJA:-`which ninja`}" || die "ninja build tool not found"
echo "Ninja found at $NINJA"

CXX="${CXX:-`which c++`}" || CXX=`which clang++` || CXX=`which g++` || die "C++ compile not found"

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
        ;;
    clang*)
        # clang version X.Y.Z (...)
        CLANG_VERSION=${CXX_VERSION#*version } # remove prefix up to 'version'
        CLANG_VERSION=${CLANG_VERSION%% *} # remove suffix after space
        echo "LLVM C++ compiler $CLANG_VERSION found at $CXX"
        if [[ $CLANG_VERSION = [0-4].* ]] ; then
            echo "WARNING: compiler version too old.  Build will fail."
        fi
        ;;
    *)
        echo "WARNING: unknown compiler.  Build may fail."
        ;;
esac

PKG_CONFIG="${PKG_CONFIG:-`which pkg-config`}" || die "pkg-config not found"
echo "pkg-config found at $PKG_CONFIG"

DEPS="eigen3"

VERSIONS=$($PKG_CONFIG --modversion $DEPS) || die "required dependency is missing"

DEPS_ALL=$((printf "%s\n" $DEPS ; $PKG_CONFIG --print-requires $DEPS) | sort | uniq)

CFLAGS="${CFLAGS:--O3 -march=native}"
CFLAGS+=" -std=c++17 -I../src -I../../nigh/src"
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
	  command = \$cxx -MMD -MT \$out -MF \$out.d \$cflags \$in -o \$out \$libs
	  description = CXX \$out
	  depfile = \$out.d
	  deps = gcc

	rule test
	  command = \$in && touch \$out
	  description = TEST \$in

	EOF

tests=""
for src in *_test.cpp ; do
    base="${src%.*}"
    tests+=" $base"
    cat <<-EOF >&3
	build \$builddir/$base: cxx $src
	build \$builddir/$base.success: test \$builddir/$base
	build $base: phony \$builddir/$test
	EOF
done

echo "build all: phony$tests" >&3

exec 3>&- # close build.ninja
echo "Done.  type 'ninja' to start build"
