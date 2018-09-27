#!/bin/bash

declare -a tests
for file in *_test.cpp ; do
    tests+=(${file%.*})
done

(
    cat <<EOF
ninja_required_version = 1.3
root = .
builddir = build
cxx = clang++
cflags = -std=c++17 -I../src -I../../nigh/src -I/usr/include/eigen3 -Wall -pedantic -Wno-ignored-attributes -march=native -O3
libs = -lpthread

pool concurrent_pool
  depth = 1

rule cxx
  command = \$cxx -MMD -MT \$out -MF \$out.d \$cflags \$ndebug \$in -o \$out \$libs
  description = CXX \$out
  depfile = \$out.d
  deps = gcc

rule test
  command = \$in > \$in.log && touch \$out
  description = TEST \$in

EOF

    for test in ${tests[@]} ; do
        echo "build \$builddir/$test: cxx $test.cpp"
        echo "build \$builddir/$test.success: test \$builddir/$test"
        echo "build $test: phony \$builddir/$test.success"
    done

    echo ""
    echo "build all: phony ${tests[@]}"

) > build.ninja

