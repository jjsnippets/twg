#
# Makefile — BNO085 Unified Build System for Raspberry Pi 4B
#
# Production app, calibration, orientation, validation harnesses, and tests.
#
# Layout:
#   app/         production application + Raspberry Pi HAL (sh2_hal_rpi.c)
#   calibration/ BNO085 dynamic-calibration and DCD erase tool
#   orientation/ BNO085 swing-axis tare tool
#   validation/  AMT102 encoder vs BNO085 synchronized validation harness
#   sh2/         vendored CEVA SH-2 sensor hub library
#   tests/       hardware diagnostics and unit test suite
#   bin/         (generated) binary deliverables
#   build/       (generated) intermediate object files
#
# Targets:
#   make                build bin/bno_app (default)
#   make app            build bin/bno_app
#   make cal            build bin/bno_cal
#   make orient         build bin/bno_orient
#   make validate       build bin/bno_validate
#   make bringup        build bin/amt102_bringup
#   make tools          build all production & field tools
#   make tests          build all hardware diagnostics and unit tests
#   make test           build and execute host unit test (bin/test_quad_decode)
#   make all            build all tools and test binaries
#   make clean          remove build/ and bin/
#

SH2DIR    := sh2
APPDIR    := app
CALDIR    := calibration
ORIENTDIR := orientation
VALDIR    := validation
TESTDIR   := tests
BUILDDIR  := build
BINDIR    := bin

CC       := gcc
CFLAGS   := -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE
CPPFLAGS := -I. -I$(SH2DIR) -I$(APPDIR) -I$(CALDIR) -I$(ORIENTDIR) -I$(VALDIR)
LDLIBS   := -lgpiod -lm -lpthread

# Vendored CEVA SH-2 objects
SH2SRCS := \
	$(SH2DIR)/sh2.c \
	$(SH2DIR)/shtp.c \
	$(SH2DIR)/sh2_util.c \
	$(SH2DIR)/sh2_SensorValue.c \
	$(SH2DIR)/euler.c
SH2OBJS := $(patsubst $(SH2DIR)/%.c,$(BUILDDIR)/sh2_%.o,$(SH2SRCS))

# Shared Raspberry Pi HAL and Realtime support objects
APPHALOBJ := $(BUILDDIR)/app_sh2_hal_rpi.o
APPRTOBJ  := $(BUILDDIR)/app_realtime.o

# App objects
APPOBJS := $(BUILDDIR)/app_app_sensor.o $(BUILDDIR)/app_main.o

# Calibration objects
CALOBJS := $(BUILDDIR)/cal_cal_sensor.o $(BUILDDIR)/cal_cal_main.o

# Orientation objects
ORIENTOBJS := $(BUILDDIR)/orient_orient_sensor.o $(BUILDDIR)/orient_orient_main.o

# Validation objects
VALOBJS := \
	$(BUILDDIR)/val_amt102.o \
	$(BUILDDIR)/val_quad_decode.o \
	$(BUILDDIR)/val_validate_logger.o \
	$(BUILDDIR)/val_validate_sensor.o \
	$(BUILDDIR)/val_validate_main.o

BRINGUPOBJS := \
	$(BUILDDIR)/val_amt102_bringup.o \
	$(BUILDDIR)/val_amt102.o \
	$(BUILDDIR)/val_quad_decode.o

# Test suite sources
TESTHALRAWSRC       := $(TESTDIR)/test_hal_raw.c
TESTSH2OPENSRC      := $(TESTDIR)/test_sh2_open.c
TESTSINGLESENSORSRC := $(TESTDIR)/test_single_sensor.c
TESTMINPERIODSRC    := $(TESTDIR)/test_min_period.c
TESTREPORTLENSRC    := $(TESTDIR)/test_report_len.c
TESTQDSRC           := $(TESTDIR)/test_quad_decode.c

.PHONY: default all app cal orient validate bringup tools tests test clean

default: $(BINDIR)/bno_app

app: $(BINDIR)/bno_app

cal: $(BINDIR)/bno_cal

orient: $(BINDIR)/bno_orient

validate: $(BINDIR)/bno_validate

bringup: $(BINDIR)/amt102_bringup

tools: $(BINDIR)/bno_app $(BINDIR)/bno_cal $(BINDIR)/bno_orient $(BINDIR)/bno_validate $(BINDIR)/amt102_bringup

all: tools tests

# ------------------------------------------------------------------
# Binaries
# ------------------------------------------------------------------

$(BINDIR)/bno_app: $(SH2OBJS) $(APPHALOBJ) $(APPRTOBJ) $(APPOBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/bno_cal: $(CALOBJS) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/bno_orient: $(ORIENTOBJS) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/bno_validate: $(VALOBJS) $(APPHALOBJ) $(APPRTOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/amt102_bringup: $(BRINGUPOBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ -lgpiod

# ------------------------------------------------------------------
# Test Binaries
# ------------------------------------------------------------------

tests: \
	$(BINDIR)/test_hal_raw \
	$(BINDIR)/test_sh2_open \
	$(BINDIR)/test_single_sensor \
	$(BINDIR)/test_min_period \
	$(BINDIR)/test_report_len \
	$(BINDIR)/test_quad_decode

test: $(BINDIR)/test_quad_decode
	$(BINDIR)/test_quad_decode

$(BINDIR)/test_quad_decode: $(TESTQDSRC) $(BUILDDIR)/val_quad_decode.o | $(BINDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^

$(BINDIR)/test_hal_raw: $(TESTHALRAWSRC) $(APPHALOBJ) | $(BINDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/test_sh2_open: $(TESTSH2OPENSRC) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/test_single_sensor: $(TESTSINGLESENSORSRC) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/test_min_period: $(TESTMINPERIODSRC) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDLIBS)

$(BINDIR)/test_report_len: $(TESTREPORTLENSRC) $(APPHALOBJ) $(SH2OBJS) $(APPRTOBJ) | $(BINDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDLIBS)

# ------------------------------------------------------------------
# Pattern rules for module objects
# ------------------------------------------------------------------

$(BUILDDIR)/sh2_%.o: $(SH2DIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR)/app_%.o: $(APPDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR)/cal_%.o: $(CALDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR)/orient_%.o: $(ORIENTDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR)/val_%.o: $(VALDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR):
	mkdir -p $@

$(BINDIR):
	mkdir -p $@

clean:
	rm -rf $(BUILDDIR) $(BINDIR)
