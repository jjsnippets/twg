#
# Makefile -- BNO085 (SH-2) bring-up, application and test programs
# for Raspberry Pi 4B.
#
# Layout:
#   sh2/   vendored CEVA sh2 library (unmodified clone of ceva-dsp/sh2)
#   app/   production application + Raspberry Pi HAL (sh2_hal_rpi.c)
#   tests/ diagnostic programs (throwaway -- not part of the final app)
#
# Targets:
#   make               build bin/bno_app
#   make tests         build every test program
#   make test_X        build a single test program
#   make clean         remove build/ and bin/
#

SH2DIR := sh2
APPDIR := app
TESTDIR := tests
BUILDDIR := build
BINDIR := bin

CC := gcc
CFLAGS := -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE
INCLUDES := -I$(SH2DIR) -I$(APPDIR)
LDLIBS := -lgpiod -lm -lpthread

SH2SRCS := \
	$(SH2DIR)/sh2.c \
	$(SH2DIR)/shtp.c \
	$(SH2DIR)/sh2_util.c \
	$(SH2DIR)/sh2_SensorValue.c \
	$(SH2DIR)/euler.c

SH2OBJS := $(patsubst $(SH2DIR)/%.c,$(BUILDDIR)/sh2_%.o,$(SH2SRCS))

APPHALSRC := $(APPDIR)/sh2_hal_rpi.c
APPHALOBJ := $(BUILDDIR)/app_sh2_hal_rpi.o

APPLOGICSRCS := \
	$(APPDIR)/sensor_reader.c \
	$(APPDIR)/realtime.c

APPLOGICOBJS := \
	$(BUILDDIR)/app_sensor_reader.o \
	$(BUILDDIR)/app_realtime.o

APPMAINSRC := $(APPDIR)/main.c
APPMAINOBJ := $(BUILDDIR)/app_main.o

TESTHALRAWSRC := $(TESTDIR)/test_hal_raw.c
TESTSH2OPENSRC := $(TESTDIR)/test_sh2_open.c
TESTSINGLESENSORSRC := $(TESTDIR)/test_single_sensor.c
TESTMINPERIODSRC := $(TESTDIR)/test_min_period.c
TESTREPORTLENSRC := $(TESTDIR)/test_report_len.c

.PHONY: all
all: $(BINDIR)/bno_app

$(BINDIR)/bno_app: $(SH2OBJS) $(APPHALOBJ) $(APPLOGICOBJS) $(APPMAINOBJ) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BUILDDIR)/sh2_%.o: $(SH2DIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(BUILDDIR)/app_%.o: $(APPDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(BUILDDIR):
	mkdir -p $@

$(BINDIR):
	mkdir -p $@

.PHONY: tests
tests: \
	$(BINDIR)/test_hal_raw \
	$(BINDIR)/test_sh2_open \
	$(BINDIR)/test_single_sensor \
	$(BINDIR)/test_min_period \
	$(BINDIR)/test_report_len

.PHONY: test_hal_raw
test_hal_raw: $(BINDIR)/test_hal_raw

$(BINDIR)/test_hal_raw: $(TESTHALRAWSRC) $(APPHALOBJ) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDLIBS)

.PHONY: test_sh2_open
test_sh2_open: $(BINDIR)/test_sh2_open

$(BINDIR)/test_sh2_open: $(TESTSH2OPENSRC) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDLIBS)

.PHONY: test_single_sensor
test_single_sensor: $(BINDIR)/test_single_sensor

$(BINDIR)/test_single_sensor: $(TESTSINGLESENSORSRC) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDLIBS)

.PHONY: test_min_period
test_min_period: $(BINDIR)/test_min_period

$(BINDIR)/test_min_period: $(TESTMINPERIODSRC) $(APPHALOBJ) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDLIBS)

.PHONY: test_report_len
test_report_len: $(BINDIR)/test_report_len

$(BINDIR)/test_report_len: $(TESTREPORTLENSRC) $(APPHALOBJ) $(SH2OBJS) $(BUILDDIR)/app_realtime.o | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDLIBS)

.PHONY: clean
clean:
	rm -rf $(BUILDDIR) $(BINDIR)