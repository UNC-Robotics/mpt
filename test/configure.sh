#!/bin/bash

die () { echo "FATAL: $1" ; exit 1; }

NINJA="${NINJA:-`which ninja`}" || die "ninja build tool not found"
echo "Ninja found at $NINJA"

CXX="${CXX:-`which c++`}" || CXX=`which clang++` || CXX=`which g++` || die "C++ compile not found"
echo "C++ compiler found at $CXX"

PKG_CONFIG="${PKG_CONFIG:-`which pkg-config`}" || die "pkg-config not found"
echo "pkg-config found at $PKG_CONFIG"

DEPS="eigen3"

VERSIONS=$($PKG_CONFIG --modversion $DEPS) || die "required dependency is missing"

DEPS_ALL=$((printf "%s\n" $DEPS ; $PKG_CONFIG --print-requires $DEPS) | sort | uniq)

CFLAGS="${CFLAGS:--O3 -march=native}"
CFLAGS+=" -std=c++17 -I../src -I../../nigh/src"
CFLAGS+=" $($PKG_CONFIG --cflags-only-I $DEPS_ALL)"
LIBS="$($PKG_CONFIG --libs eigen3 $DEPS_ALL)"

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
