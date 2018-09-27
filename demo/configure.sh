#!/bin/bash

die () { echo "FATAL: $1" ; exit 1; }

NINJA="${NINJA:-`which ninja`}" || die "ninja build tool not found"
echo "Ninja found at $NINJA"

CXX="${CXX:-`which c++`}" || CXX=`which clang++` || CXX=`which g++` || die "C++ compile not found"
echo "C++ compiler found at $CXX"

PKG_CONFIG="${PKG_CONFIG:-`which pkg-config`}" || die "pkg-config not found"
echo "pkg-config found at $PKG_CONFIG"

DEPS="eigen3 fcl assimp"

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
