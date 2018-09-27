SHELL = bash

# We prefer clang++, GMake prefers g++
ifeq ($(origin CXX),default)
  CXX = clang++
endif

CXXFLAGS += -std=c++17 -I../src -Wall -pedantic -Wno-ignored-attributes -lpthread

CXXFLAGS += -I../../nigh/src -fopenmp

# For Eigen in GCC
CXXFLAGS += -Wno-int-in-bool-context
# For the above flag in clang
CXXFLAGS += -Wno-unknown-warning-option

# Use libstdc++ on Linux, and libc++ on 
# UNAME_S := $(shell uname -s)
# ifeq ($(UNAME_S),Linux)
# CXXFLAGS += -stdlib=libstdc++ -lpthread
# else ifeq ($(UNAME_S),Darwin)
# CXXFLAGS += -stdlib=libc++
# endif

# g++-7 complains generates warnings in Eigen without the following:
# CXXFLAGS += -Wno-int-in-bool-context

# Make sure to set the benchmarking flags BEFORE modifying CXXFLAGS to
# include the BUILD_TYPE-based flags.
CXXBENCHFLAGS = $(CXXFLAGS) -march=native -O3 -DNDEBUG

# Support for different types of test builds.  These BUILD_TYPE
# variable does NOT effect benchmark compilation.
BUILD_TYPE ?= release
ifeq ($(BUILD_TYPE),release)
CXXFLAGS += -march=native -O3 -DBUILD_TYPE_RELEASE
else ifeq ($(BUILD_TYPE),debug)
CXXFLAGS += -march=native -g -DBUILD_TYPE_DEBUG
else ifeq ($(BUILD_TYPE),valgrind)
CXXFLAGS += -march=native -O0 -g -DBUILD_TYPE_VALGRIND
else
$(error Bad value for BUILD_TYPE.  Must be one of 'release', 'debug', or 'valgrind')
endif

# it would be preferable to do something like this, but gmake does not
# include a simple toupper solution.  So we include the
# -DBUILD_TYPE_XXX above.
#
# If we add GNU make standard library (http://gmsl.sourceforge.net/),
# then we can use the 'uc' function.
#
# CXXFLAGS += -DBUILD_TYPE_$(toupper $(BUILD_TYPE))

BUILD_ROOT = build/$(BUILD_TYPE)
BUILD_BENCH_ROOT = build/bench

# UNAME_S := $(shell uname -s)
# ifeq ($(UNAME_S),Linux)
# CXXFLAGS += -stdlib=libstdc++ -lpthread
# else # ifeq ($(UNAME_S),Darwin)
# CXXFLAGS += -stdlib=libc++
# endif

PKG_CONFIG ?= pkg-config

ifeq ($(origin EIGEN_INCLUDE), undefined)
  ifeq ($(shell $(PKG_CONFIG) --exists eigen3 && echo 1),1)
    CXXFLAGS += $(shell $(PKG_CONFIG) --cflags eigen3)
  endif
else
  CXXFLAGS += -I$(EIGEN_INCLUDE)
endif

testsrc := $(wildcard *_test.cpp)

# Declare testrun to be all the tests to run.  These are simply the
# *_test.cpp files with the .cpp removed.  This allows us to type
#    make foo_test
# to run the test in foo_test.cpp
testrun := $(patsubst %.cpp,%,$(testsrc))

.PHONY: all clean $(testrun)
all: $(testrun)

clean:
	rm -rf build

# remove default rule
%: %.cpp

# Compile _test.cpp files directly to executable (no intermediate .o)
.PRECIOUS: $(patsubst %.cpp,$(BUILD_ROOT)/bin/%,$(testsrc))
$(BUILD_ROOT)/bin/%: %.cpp $(BUILD_ROOT)/dep/%.d
	@echo "Compiling" $@
	@mkdir -p $(@D)
	@mkdir -p $(BUILD_ROOT)/dep
	@$(CXX) -MMD -MF $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.dtmp,$<) $(CXXFLAGS) -o $@ $<
	@mv $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.dtmp,$<) $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.d,$<)
	@touch $@

# Runs a test.  On success it touches the .success file.  Either way it writes the .log file.
$(BUILD_ROOT)/log/%.success: $(BUILD_ROOT)/bin/%
	@echo "Running test" $<
	@mkdir -p $(@D)
	@set -o pipefail ; $< | tee $(BUILD_ROOT)/log/$*.log && touch $@

# Adding .NOTPARALLEL disable parallelism in ALL targets instead of
# just the listed ones.  This seems like a bug, though it is somewhat
# consistent with documentation.
#
# .NOTPARALLEL: $(BUILD_ROOT)/log/%_parallel_test.success
$(BUILD_ROOT)/log/%_parallel_test.success: $(BUILD_ROOT)/bin/%_parallel_test
	@echo "Running parallel test" $<
	@mkdir -p $(@D)
	@set -o pipefail ; $< | tee $(BUILD_ROOT)/log/$*.log && touch $@


# $(info testrun is $(testrun))

# Static pattern rule for al the test files to run
$(testrun): %: $(BUILD_ROOT)/log/%.success

# comma variable so that we can subst in a comma
comma:=,
# $(BUILD_ROOT)/bin/bench-L2-double-3-20-Concurrent-BATCH8-10000
.PRECIOUS: $(BUILD_BENCH_ROOT)/bin/bench-%
$(BUILD_BENCH_ROOT)/bin/bench-%: bench.cpp
	@echo "Compiling benchmark $@"
	@mkdir -p $(@D)
	@mkdir -p $(BUILD_BENCH_ROOT)/dep
	@$(CXX) -MMD -MF $(patsubst $(BUILD_BENCH_ROOT)/bin/%,$(BUILD_BENCH_ROOT)/dep/%.dtmp,$@) $(CXXBENCHFLAGS) -DSPEC=$(subst -,$(comma),$(patsubst $(BUILD_BENCH_ROOT)/bin/bench-%,%,$@)) -o $@ $<
	@mv $(patsubst $(BUILD_BENCH_ROOT)/bin/%,$(BUILD_BENCH_ROOT)/dep/%.dtmp,$@) $(patsubst $(BUILD_BENCH_ROOT)/bin/%,$(BUILD_BENCH_ROOT)/dep/%.d,$@)
	@touch $@

.PRECIOUS: $(BUILD_BENCH_ROOT)/data/%.dat
$(BUILD_BENCH_ROOT)/data/%.dat: $(BUILD_BENCH_ROOT)/bin/bench-%
	@mkdir -p $(BUILD_BENCH_ROOT)/data
	$< > $@.tmp
	@mv $@.tmp $@

# Example:
#   plot-L2-double-3-20-50-Concurrent:
# L2 space with double scalars, 3 dimensions, k=20 for nearest neighbor, 50 steps per exponent, concurrent versions
.PHONY: plot-%.qt plot-%.aqua
# plot-%: $(BUILD_BENCH_ROOT)/data/%-LINEAR-3000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-VALUE-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-MIDPOINT-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-MEDIAN-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-BATCH4-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-BATCH8-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-BATCH16-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-GNAT-1000000.dat
# 	./plot.sh $@ $?

# plot-%: $(BUILD_BENCH_ROOT)/data/%-LINEAR-3000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-MIDPOINT-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-BATCH4-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-BATCH8-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-BATCH16-1000000.dat \
# 	$(BUILD_BENCH_ROOT)/data/%-GNAT-1000000.dat
# 	./plot.sh $@ $?

# SECONDEXPANSION is needed for the plot-... rules to make the $$(basename %) work.
.SECONDEXPANSION:

# SO2 isn't supported by value and median yet, so we have a special rule just for those plots
plot-SO2-%: $(BUILD_BENCH_ROOT)/data/SO2-$$(basename %)-LINEAR-100000.dat \
	$(foreach strat,MIDPOINT BATCH8 BATCH16 GNAT,$(BUILD_BENCH_ROOT)/data/SO2-$$(basename %)-$(strat)-1000000.dat)
	./plot.sh $@ $?

plot-SE2-%: $(BUILD_BENCH_ROOT)/data/SE2-$$(basename %)-LINEAR-100000.dat \
	$(foreach strat,MIDPOINT BATCH8 BATCH16 GNAT,$(BUILD_BENCH_ROOT)/data/SE2-$$(basename %)-$(strat)-1000000.dat)
	./plot.sh $@ $?

#
plot-NSPHERE-%: \
	$(foreach strat,BATCH2 GNAT,$(BUILD_BENCH_ROOT)/data/NSPHERE-$$(basename %)-$(strat)-1000000.dat) \
	$(BUILD_BENCH_ROOT)/data/NSPHERERL-$$(basename %)-BATCH2-1000000.dat
	./plot.sh $@ $?

plot-%: $(BUILD_BENCH_ROOT)/data/$$(basename %)-LINEAR-100000.dat \
	$(foreach strat,MIDPOINT VALUE MEDIAN BATCH8 BATCH16 GNAT,$(BUILD_BENCH_ROOT)/data/$$(basename %)-$(strat)-1000000.dat)
	./plot.sh $@ $?

.PHONY: site-plots
site-plots: plot-SO2-double-6-20-200-ConcurrentRead.svg \
	    plot-SO3-double-3-20-200-ConcurrentRead.svg \
	    plot-SE2-double-3-20-200-ConcurrentRead.svg \
	    plot-SE3-double-6-20-200-ConcurrentRead.svg \
	    plot-L2-double-3-20-200-ConcurrentRead.svg \
	    plot-L2-double-6-20-200-ConcurrentRead.svg

%.d: ;
.PRECIOUS: %.d

# Include generated dependencies
-include $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.d,$(testsrc))
-include $(wildcard $(BUILD_BENCH_ROOT)/dep/bench-*.d)
