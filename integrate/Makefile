CC ?= gcc

CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c11
CPPFLAGS ?= -D_GNU_SOURCE
LDFLAGS ?=
LDLIBS ?= -lgpiod -lpthread -lm -lrt

ROOT := ..
BNO := $(ROOT)/bno
BNO_APP := $(BNO)/app
SH2 := $(BNO)/sh2
MS5_APP := $(ROOT)/ms5/app
RT := $(ROOT)/rt

BUILD := build
BIN := bin
TARGET := $(BIN)/twg_app

CPPFLAGS += \
	-I$(ROOT) \
	-I$(BNO) \
	-I$(BNO_APP) \
	-I$(SH2) \
	-I$(MS5_APP)

INTEGRATE_SRCS := \
	main.c \
	twg_term.c \
	twg_log.c \
	twg_stage.c

BNO_SRCS := \
	$(BNO_APP)/imu_session.c \
	$(BNO_APP)/sh2_hal_rpi.c \
	$(BNO_APP)/imu_cmd.c \
	$(BNO_APP)/imu_cal.c \
	$(BNO_APP)/imu_cal_adapter.c \
	$(BNO_APP)/imu_tare.c \
	$(BNO_APP)/imu_tare_adapter.c \
	$(BNO_APP)/imu_check.c \
	$(BNO_APP)/imu_check_adapter.c \
	$(BNO_APP)/imu_cli.c \
	$(BNO_APP)/imu_publish.c

SH2_SRCS := \
	$(SH2)/sh2.c \
	$(SH2)/shtp.c \
	$(SH2)/sh2_util.c \
	$(SH2)/sh2_SensorValue.c \
	$(SH2)/euler.c

MS5_SRCS := \
	$(MS5_APP)/ms5837_driver.c \
	$(MS5_APP)/ms5837_hal_rpi.c \
	$(MS5_APP)/ms5837_math.c

RT_SRCS := \
	$(RT)/realtime.c

SRCS := \
	$(INTEGRATE_SRCS) \
	$(BNO_SRCS) \
	$(SH2_SRCS) \
	$(MS5_SRCS) \
	$(RT_SRCS)

OBJS := $(addprefix $(BUILD)/,$(patsubst $(ROOT)/%,ext/%,$(SRCS:.c=.o)))
DEPS := $(OBJS:.o=.d)

.PHONY: all app clean

all: app

app: $(TARGET)

$(TARGET): $(OBJS) | $(BIN)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/ext/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c -o $@ $<

$(BIN):
	mkdir -p $(BIN)

clean:
	rm -rf $(BUILD) $(BIN)

-include $(DEPS)
