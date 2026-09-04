# validation/Makefile — encoder + IMU validation harnesses
#
# Targets:
#   make            build bin/amt102_bringup, bin/test_quad_decode, bin/sensor_validate
#   make test       build and run the decoder unit test (no hardware)
#   make bringup    build only the encoder bring-up tool
#   make validate   build only the synchronized validation binary
#   make clean      remove build/ and bin/
#
# CPPFLAGS = -I. lets tests/test_quad_decode.c include the headers that
# live in this directory, and lets main.c/sensor_validate.c reach the
# vendored sh2 headers and the reused app headers. The object rules
# mkdir their own output path so out-of-tree objects (sh2_*, app_*)
# land under build/ without hand-written directory rules.
#
# sensor_validate links the production decode chain: the sh2 library
# (../sh2), the Pi HAL and RT helpers (../app), and the validated
# encoder modules in this directory. It must not run at the same time
# as bno_app — one SPI HAL instance per process.

CC       = gcc
CFLAGS   = -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE
CPPFLAGS = -I. -I$(SH2DIR) -I$(APPDIR)
LDLIBS   = -lgpiod
SVLDLIBS = -lgpiod -lm -lpthread

SH2DIR   = ../sh2
APPDIR   = ../app

BUILDDIR = build
BINDIR   = bin

HEADERS = amt102.h quad_decode.h imu_validate.h sensor_validate.h csv_log.h

# bring-up tool: hardware integration test for amt102 + quad_decode
BRINGUP_SRC = amt102_bringup.c amt102.c quad_decode.c
BRINGUP_OBJ = $(patsubst %.c,$(BUILDDIR)/%.o,$(BRINGUP_SRC))

# decoder unit test: pure quad_decode, no libgpiod, no hardware
TESTQD_SRC  = tests/test_quad_decode.c quad_decode.c
TESTQD_OBJ  = $(patsubst %.c,$(BUILDDIR)/%.o,$(TESTQD_SRC))

# synchronized validation binary: 100 Hz encoder+IMU records to CSV
SH2SRCS = $(SH2DIR)/sh2.c \
          $(SH2DIR)/sh2_SensorValue.c \
          $(SH2DIR)/sh2_util.c \
          $(SH2DIR)/shtp.c \
          $(SH2DIR)/euler.c
SH2OBJS = $(patsubst $(SH2DIR)/%.c,$(BUILDDIR)/sh2_%.o,$(SH2SRCS))

APPSRCS = $(APPDIR)/sh2_hal_rpi.c \
          $(APPDIR)/realtime.c
APPOBJS = $(patsubst $(APPDIR)/%.c,$(BUILDDIR)/app_%.o,$(APPSRCS))

SVSRCS = main.c sensor_validate.c csv_log.c amt102.c quad_decode.c
SVOBJS = $(patsubst %.c,$(BUILDDIR)/%.o,$(SVSRCS))

all: $(BINDIR)/amt102_bringup $(BINDIR)/test_quad_decode $(BINDIR)/sensor_validate

bringup: $(BINDIR)/amt102_bringup

validate: $(BINDIR)/sensor_validate

test: $(BINDIR)/test_quad_decode
	$(BINDIR)/test_quad_decode

# local sources (validation/ and validation/tests/)
$(BUILDDIR)/%.o: %.c $(HEADERS)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

# vendored sh2 library sources
$(BUILDDIR)/sh2_%.o: $(SH2DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

# reused app sources (HAL transport, RT helpers)
$(BUILDDIR)/app_%.o: $(APPDIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BINDIR)/amt102_bringup: $(BRINGUP_OBJ) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(BRINGUP_OBJ) $(LDLIBS)

$(BINDIR)/test_quad_decode: $(TESTQD_OBJ) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(TESTQD_OBJ)

$(BINDIR)/sensor_validate: $(SVOBJS) $(APPOBJS) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(SVOBJS) $(APPOBJS) $(SH2OBJS) $(SVLDLIBS)

$(BINDIR):
	mkdir -p $@

clean:
	rm -rf $(BUILDDIR) $(BINDIR)

.PHONY: all bringup validate test clean
