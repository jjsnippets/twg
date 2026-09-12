CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c99 -D_GNU_SOURCE
LDFLAGS ?= -lrt -lpthread

BNO_DIR = ../bno/app
TESTDIR = tests
BUILDDIR = build
BINDIR = bin

TEST_RAW = $(BINDIR)/test_i2c_raw
TEST_MATH = $(BINDIR)/test_crc_math
TEST_BENCH = $(BINDIR)/test_rate_bench

RT_OBJ = $(BUILDDIR)/realtime.o

.PHONY: all clean dirs tests

all: dirs tests

dirs:
	@mkdir -p $(BUILDDIR) $(BINDIR)

tests: dirs $(TEST_RAW) $(TEST_MATH) $(TEST_BENCH)

# Compile realtime.o directly from the single source of truth in bno/app
$(RT_OBJ): $(BNO_DIR)/realtime.c $(BNO_DIR)/realtime.h
	$(CC) $(CFLAGS) -c $< -o $@

# Test binaries
$(BUILDDIR)/test_i2c_raw.o: $(TESTDIR)/test_i2c_raw.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_i2c_raw: $(BUILDDIR)/test_i2c_raw.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILDDIR)/test_crc_math.o: $(TESTDIR)/test_crc_math.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_crc_math: $(BUILDDIR)/test_crc_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILDDIR)/test_rate_bench.o: $(TESTDIR)/test_rate_bench.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_rate_bench: $(BUILDDIR)/test_rate_bench.o $(RT_OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILDDIR) $(BINDIR)