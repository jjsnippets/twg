CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c99 -D_GNU_SOURCE
LDFLAGS ?= -lrt -lpthread

BNO_DIR = ../bno/app
TESTDIR = tests
APPDIR = app
BUILDDIR = build
BINDIR = bin

TEST_RAW = $(BINDIR)/test_i2c_raw
TEST_MATH = $(BINDIR)/test_crc_math
TEST_BENCH = $(BINDIR)/test_rate_bench
TEST_PROD_MATH = $(BINDIR)/test_ms5837_math
TEST_REFERENCE = $(BINDIR)/test_ms5837_reference

RT_OBJ = $(BUILDDIR)/realtime.o

.PHONY: all clean dirs tests

all: dirs tests

dirs:
	@mkdir -p $(BUILDDIR) $(BINDIR)

tests: dirs $(TEST_RAW) $(TEST_MATH) $(TEST_BENCH) $(TEST_PROD_MATH) $(TEST_REFERENCE)

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

# Step 4A production calculation and surface-reference tests
$(BUILDDIR)/ms5837_math.o: | dirs
$(BUILDDIR)/ms5837_math.o: $(APPDIR)/ms5837_math.c $(APPDIR)/ms5837_math.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BUILDDIR)/ms5837_reference.o: | dirs
$(BUILDDIR)/ms5837_reference.o: $(APPDIR)/ms5837_reference.c $(APPDIR)/ms5837_reference.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BUILDDIR)/test_ms5837_math.o: | dirs
$(BUILDDIR)/test_ms5837_math.o: $(TESTDIR)/test_ms5837_math.c $(APPDIR)/ms5837_math.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BINDIR)/test_ms5837_math: $(BUILDDIR)/test_ms5837_math.o $(BUILDDIR)/ms5837_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

$(BUILDDIR)/test_ms5837_reference.o: | dirs
$(BUILDDIR)/test_ms5837_reference.o: $(TESTDIR)/test_ms5837_reference.c $(APPDIR)/ms5837_reference.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BINDIR)/test_ms5837_reference: $(BUILDDIR)/test_ms5837_reference.o $(BUILDDIR)/ms5837_reference.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

clean:
	rm -rf $(BUILDDIR) $(BINDIR)
