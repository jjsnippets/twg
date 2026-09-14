CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c99 -D_GNU_SOURCE
LDFLAGS ?= -lrt -lpthread

REPODIR = ..
RT_DIR = $(REPODIR)/rt
TESTDIR = tests
APPDIR = app
BUILDDIR = build
BINDIR = bin

TEST_RAW = $(BINDIR)/test_i2c_raw
TEST_MATH = $(BINDIR)/test_crc_math
TEST_BENCH = $(BINDIR)/test_rate_bench
TEST_PROD_MATH = $(BINDIR)/test_ms5837_math
TEST_REFERENCE = $(BINDIR)/test_ms5837_reference
TEST_INIT_MOCK = $(BINDIR)/test_ms5837_init_mock
TEST_INIT_LIVE = $(BINDIR)/test_ms5837_init_live
TEST_ACQUISITION_MOCK = $(BINDIR)/test_ms5837_acquisition_mock
TEST_ACQUISITION_LIVE = $(BINDIR)/test_ms5837_acquisition_live
TEST_APP = $(BINDIR)/test_ms5837_app
CAPTURE_APP = $(BINDIR)/ms5837_capture

RT_OBJ = $(BUILDDIR)/realtime.o

.PHONY: all clean dirs tests host-tests live-tests app

all: dirs tests app

dirs:
	@mkdir -p $(BUILDDIR) $(BINDIR)

tests: dirs $(TEST_RAW) $(TEST_MATH) $(TEST_BENCH) $(TEST_PROD_MATH) $(TEST_REFERENCE) $(TEST_INIT_MOCK) $(TEST_INIT_LIVE) $(TEST_ACQUISITION_MOCK) $(TEST_ACQUISITION_LIVE) $(TEST_APP)

host-tests: dirs $(TEST_PROD_MATH) $(TEST_REFERENCE) $(TEST_INIT_MOCK) $(TEST_ACQUISITION_MOCK) $(TEST_APP)

live-tests: dirs $(TEST_INIT_LIVE) $(TEST_ACQUISITION_LIVE)

app: dirs $(CAPTURE_APP)

# Compile realtime.o from the repository-level lab timing helper.
$(RT_OBJ): $(RT_DIR)/realtime.c $(RT_DIR)/realtime.h
	$(CC) $(CFLAGS) -I$(REPODIR) -c $< -o $@

# Test binaries
$(BUILDDIR)/test_i2c_raw.o: $(TESTDIR)/test_i2c_raw.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/test_i2c_raw: $(BUILDDIR)/test_i2c_raw.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Archived Step 2 source intentionally remains unchanged; suppress only its
# legacy ignored-I/O-result diagnostics so the aggregate build can include it.
$(BUILDDIR)/test_crc_math.o: $(TESTDIR)/test_crc_math.c
	$(CC) $(CFLAGS) -Wno-unused-result -c $< -o $@

$(BINDIR)/test_crc_math: $(BUILDDIR)/test_crc_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Archived Step 3 benchmark intentionally remains unchanged.
$(BUILDDIR)/test_rate_bench.o: $(TESTDIR)/test_rate_bench.c
	$(CC) $(CFLAGS) -I$(REPODIR) -Wno-unused-result -c $< -o $@

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

# Step 4B generic HAL, Raspberry Pi I2C backend, and initialization tests
$(BUILDDIR)/ms5837_driver.o: | dirs
$(BUILDDIR)/ms5837_driver.o: $(APPDIR)/ms5837_driver.c $(APPDIR)/ms5837_driver.h $(APPDIR)/ms5837_hal.h $(APPDIR)/ms5837_math.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BUILDDIR)/ms5837_hal_rpi.o: | dirs
$(BUILDDIR)/ms5837_hal_rpi.o: $(APPDIR)/ms5837_hal_rpi.c $(APPDIR)/ms5837_hal_rpi.h $(APPDIR)/ms5837_hal.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BUILDDIR)/test_ms5837_init_mock.o: | dirs
$(BUILDDIR)/test_ms5837_init_mock.o: $(TESTDIR)/test_ms5837_init_mock.c $(APPDIR)/ms5837_driver.h $(APPDIR)/ms5837_hal.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(TEST_INIT_MOCK): $(BUILDDIR)/test_ms5837_init_mock.o $(BUILDDIR)/ms5837_driver.o $(BUILDDIR)/ms5837_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

$(BUILDDIR)/test_ms5837_init_live.o: | dirs
$(BUILDDIR)/test_ms5837_init_live.o: $(TESTDIR)/test_ms5837_init_live.c $(APPDIR)/ms5837_driver.h $(APPDIR)/ms5837_hal_rpi.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(TEST_INIT_LIVE): $(BUILDDIR)/test_ms5837_init_live.o $(BUILDDIR)/ms5837_driver.o $(BUILDDIR)/ms5837_hal_rpi.o $(BUILDDIR)/ms5837_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

# Step 4C fixed symmetric OSR-512 acquisition and recovery tests
$(BUILDDIR)/test_ms5837_acquisition_mock.o: | dirs
$(BUILDDIR)/test_ms5837_acquisition_mock.o: $(TESTDIR)/test_ms5837_acquisition_mock.c $(APPDIR)/ms5837_driver.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(TEST_ACQUISITION_MOCK): $(BUILDDIR)/test_ms5837_acquisition_mock.o $(BUILDDIR)/ms5837_driver.o $(BUILDDIR)/ms5837_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

$(BUILDDIR)/test_ms5837_acquisition_live.o: | dirs
$(BUILDDIR)/test_ms5837_acquisition_live.o: $(TESTDIR)/test_ms5837_acquisition_live.c $(APPDIR)/ms5837_driver.h $(APPDIR)/ms5837_hal_rpi.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(TEST_ACQUISITION_LIVE): $(BUILDDIR)/test_ms5837_acquisition_live.o $(BUILDDIR)/ms5837_driver.o $(BUILDDIR)/ms5837_hal_rpi.o $(BUILDDIR)/ms5837_math.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

# Step 4D production orchestration, restricted CLI, and asynchronous CSV logger
$(BUILDDIR)/ms5837_cli.o: | dirs
$(BUILDDIR)/ms5837_cli.o: $(APPDIR)/ms5837_cli.c $(APPDIR)/ms5837_cli.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BUILDDIR)/ms5837_logger.o: | dirs
$(BUILDDIR)/ms5837_logger.o: $(APPDIR)/ms5837_logger.c $(APPDIR)/ms5837_logger.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(BUILDDIR)/ms5837_main.o: | dirs
$(BUILDDIR)/ms5837_main.o: $(APPDIR)/main.c $(APPDIR)/ms5837_cli.h $(APPDIR)/ms5837_driver.h $(APPDIR)/ms5837_hal_rpi.h $(APPDIR)/ms5837_logger.h $(APPDIR)/ms5837_reference.h $(RT_DIR)/realtime.h
	$(CC) $(CFLAGS) -I$(APPDIR) -I$(REPODIR) -c $< -o $@

$(CAPTURE_APP): $(BUILDDIR)/ms5837_main.o $(BUILDDIR)/ms5837_cli.o $(BUILDDIR)/ms5837_logger.o $(BUILDDIR)/ms5837_driver.o $(BUILDDIR)/ms5837_hal_rpi.o $(BUILDDIR)/ms5837_math.o $(BUILDDIR)/ms5837_reference.o $(RT_OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

$(BUILDDIR)/test_ms5837_app.o: | dirs
$(BUILDDIR)/test_ms5837_app.o: $(TESTDIR)/test_ms5837_app.c $(APPDIR)/ms5837_cli.h $(APPDIR)/ms5837_logger.h $(APPDIR)/app_contract.h
	$(CC) $(CFLAGS) -I$(APPDIR) -c $< -o $@

$(TEST_APP): $(BUILDDIR)/test_ms5837_app.o $(BUILDDIR)/ms5837_cli.o $(BUILDDIR)/ms5837_logger.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lm

clean:
	rm -rf $(BUILDDIR) $(BINDIR)
