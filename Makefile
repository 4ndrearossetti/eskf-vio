# Makefile
CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -Isrc
LIBS    = -lm

# Dataset path
DATA    = ~/datasets/euroc/MH_01_easy/mav0/imu0/data.csv

# Sources: everything in src/, main binary from all of them
SRCS    = $(wildcard src/*.c)
OBJS    = $(patsubst src/%.c,build/%.o,$(SRCS))

# Module objects only (no main.o) — linked into every test
MOD_OBJS  = $(filter-out build/main.o,$(OBJS))

# Tests: every tests/*.c becomes its own binary build/test_*
TEST_SRCS = $(wildcard tests/*.c)
TEST_BINS = $(patsubst tests/%.c,build/%,$(TEST_SRCS))

all: build/main

build:
	mkdir -p build

build/main: $(OBJS)
	$(CC) $(OBJS) -o $@ $(LIBS)

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/%: tests/%.c $(MOD_OBJS) | build
	$(CC) $(CFLAGS) $< $(MOD_OBJS) -o $@ $(LIBS)

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "== $$t"; $$t || exit 1; done

run: build/main
	@./build/main $(DATA)

clean:
	rm -rf build

.PHONY: all test run clean

