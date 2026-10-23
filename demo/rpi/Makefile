CC ?= gcc

CFLAGS ?= -O2 -Wall -Wextra -Werror -pedantic -std=c11
CPPFLAGS ?= -D_GNU_SOURCE
LDFLAGS ?=
LDLIBS ?= -lgpiod -lpthread -lm -lrt

ROOT := ../..
BNO := $(ROOT)/bno
BNO_APP := $(BNO)/app
SH2 := $(BNO)/sh2
RT := $(ROOT)/rt

BUILD := build
BIN := bin
TARGET := $(BIN)/imu_demo

CPPFLAGS += \
	-I$(ROOT) \
	-I$(BNO) \
	-I$(BNO_APP) \
	-I$(SH2)

DEMO_SRCS := \
	main.c \
	demo_imu.c \
	demo_link.c

EXTERNAL_SRCS := \
	$(BNO_APP)/imu_session.c \
	$(BNO_APP)/sh2_hal_rpi.c \
	$(SH2)/sh2.c \
	$(SH2)/shtp.c \
	$(SH2)/sh2_util.c \
	$(SH2)/sh2_SensorValue.c \
	$(SH2)/euler.c \
	$(RT)/realtime.c

DEMO_OBJS := $(addprefix $(BUILD)/,$(DEMO_SRCS:.c=.o))
EXTERNAL_OBJS := $(addprefix $(BUILD)/ext/,$(patsubst $(ROOT)/%,%,$(EXTERNAL_SRCS:.c=.o)))
OBJS := $(DEMO_OBJS) $(EXTERNAL_OBJS)
DEPS := $(OBJS:.o=.d)

.PHONY: all clean check-deps

all: check-deps $(TARGET)

check-deps:
	@test -f "$(SH2)/sh2.c" || { \
		echo "Missing SH-2 submodule. Run git submodule update --init --recursive from the repository root."; \
		exit 1; \
	}

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
