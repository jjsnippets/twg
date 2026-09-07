# calibration/Makefile — BNO085 dynamic-calibration and orientation tools
#
# Targets:
#   make            build bin/bno_cal, bin/bno_cal_clear and bin/bno_orient
#   make clear      build only bin/bno_cal_clear (DCD wipe tool)
#   make orient     build only bin/bno_orient (swing-axis tare tool)
#   make clean      remove build/ and bin/
#
# bno_cal       is the guided calibration tool.
# bno_cal_clear erases ALL dynamic calibration data (flash + RAM) for
#               documenting the uncalibrated baseline; it prompts for
#               confirmation before erasing.
# bno_orient    aligns the fused output frame to the fixture's swing
#               axis with the SH-2 tare commands (independent of the
#               DCD); --persist writes it to flash.
#
# bno_cal and bno_cal_clear share the sensor_calibrate session owner;
# bno_orient uses its own (sensor_orient.c, which decodes the full
# rotation-vector quaternion). All link the production decode chain:
# the sh2 library (../sh2) and the Pi HAL (../app). None may run at
# the same time as bno_app or the validation binaries — one SPI HAL
# instance per process.

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

# cal_clear reuses the sensor_calibrate session owner (open, service,
# stop, getLatestSample) for its before/after state watches.
CLEARSRC = cal_clear.c sensor_calibrate.c
CLEAROBJ = $(patsubst %.c,$(BUILDDIR)/%.o,$(CLEARSRC))

# bno_orient uses its own session owner because it needs the full
# rotation-vector quaternion (CalSample_t carries accuracies only).
ORIENTSRC = orient_main.c sensor_orient.c
ORIENTOBJ = $(patsubst %.c,$(BUILDDIR)/%.o,$(ORIENTSRC))

.PHONY: all clear orient clean
all: $(BINDIR)/bno_cal $(BINDIR)/bno_cal_clear $(BINDIR)/bno_orient

clear: $(BINDIR)/bno_cal_clear

orient: $(BINDIR)/bno_orient

$(BINDIR)/bno_cal: $(CALOBJ) $(APPOBJS) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(CALOBJ) $(APPOBJS) $(SH2OBJS) $(LDLIBS)

$(BINDIR)/bno_cal_clear: $(CLEAROBJ) $(APPOBJS) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(CLEAROBJ) $(APPOBJS) $(SH2OBJS) $(LDLIBS)

$(BINDIR)/bno_orient: $(ORIENTOBJ) $(APPOBJS) $(SH2OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(ORIENTOBJ) $(APPOBJS) $(SH2OBJS) $(LDLIBS)

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
