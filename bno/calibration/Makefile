# calibration/Makefile — guided BNO085 dynamic-calibration tool (bno_cal)
#
# Targets:
#   make            build bin/bno_cal
#   make clean      remove build/ and bin/
#
# bno_cal links the production decode chain: the sh2 library (../sh2)
# and the Pi HAL (../app). It must not run at the same time as bno_app
# or the validation binaries — one SPI HAL instance per process.

CC       = gcc
CFLAGS   = -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE
CPPFLAGS = -I. -I$(SH2DIR) -I$(APPDIR)
LDLIBS   = -lgpiod -lm

SH2DIR   = ../sh2
APPDIR   = ../app

BUILDDIR = build
BINDIR   = bin

SH2SRCS = $(SH2DIR)/sh2.c \
          $(SH2DIR)/sh2_SensorValue.c \
          $(SH2DIR)/sh2_util.c \
          $(SH2DIR)/shtp.c \
          $(SH2DIR)/euler.c
SH2OBJS = $(patsubst $(SH2DIR)/%.c,$(BUILDDIR)/sh2_%.o,$(SH2SRCS))

APPSRCS = $(APPDIR)/sh2_hal_rpi.c
APPOBJS = $(patsubst $(APPDIR)/%.c,$(BUILDDIR)/app_%.o,$(APPSRCS))

CALSRC  = cal_main.c sensor_calibrate.c
CALOBJ  = $(patsubst %.c,$(BUILDDIR)/%.o,$(CALSRC))

.PHONY: all clean
all: $(BINDIR)/bno_cal

$(BINDIR)/bno_cal: $(CALOBJ) $(APPOBJS) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(CALOBJ) $(APPOBJS) $(SH2OBJS) $(LDLIBS)

$(BUILDDIR)/%.o: %.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR)/sh2_%.o: $(SH2DIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR)/app_%.o: $(APPDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BUILDDIR) $(BINDIR):
	mkdir -p $@

clean:
	rm -rf $(BUILDDIR) $(BINDIR)
